#include <memory>
#include <string>
#include <fstream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <map>
#include <yaml-cpp/yaml.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/accel.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "nlohmann/json.hpp"

using json = nlohmann::json;
using namespace std::chrono_literals;

class PurePursuitNode : public rclcpp::Node {
public:
    PurePursuitNode() : Node("pure_pursuit_node") {
        const char* env_domain = std::getenv("ROS_DOMAIN_ID");
        domain_id_ = (env_domain != nullptr) ? std::atoi(env_domain) : 1;

        if (!init_config_and_paths()) {
            RCLCPP_ERROR(this->get_logger(), "초기화 실패");
            return;
        }

        setup_conflict_rules();

        auto qos = rclcpp::SensorDataQoS();
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/Ego_pose", qos, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                this->curr_x_ = msg->pose.position.x;
                this->curr_y_ = msg->pose.position.y;
                this->curr_yaw_ = msg->pose.orientation.z;
                this->pose_received_ = true;
            });

        // CAV 1-4 구독
        for (int i = 1; i <= 4; ++i) {
            if (i == (int)domain_id_) continue;
            std::string topic = "/CAV" + std::to_string(i) + "_pose";
            other_subs_[i] = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                topic, qos, [this, i](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                    this->others_pos_[i] = {msg->pose.position.x, msg->pose.position.y};
                    this->others_received_[i] = true;
                });
        }

        // HV 구독
        other_subs_[19] = this->create_subscription<geometry_msgs::msg::PoseStamped>("/HV_19", qos, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { this->others_pos_[19] = {msg->pose.position.x, msg->pose.position.y}; this->others_received_[19] = true; });
        other_subs_[20] = this->create_subscription<geometry_msgs::msg::PoseStamped>("/HV_20", qos, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { this->others_pos_[20] = {msg->pose.position.x, msg->pose.position.y}; this->others_received_[20] = true; });

        accel_pub_ = this->create_publisher<geometry_msgs::msg::Accel>("/Accel", 10);
        timer_ = this->create_wall_timer(50ms, std::bind(&PurePursuitNode::control_loop, this));
    }

private:
    struct CollisionInfo {
        bool collision_predicted = false;
        double dist_to_collision_ego = 1e9;
        double dist_to_collision_other = 1e9;
        int other_id = -1;
    };
// 내 위치 세그먼트별로 감시해야 할 타겟들을 묶는 구조체
    struct Path { std::vector<double> X, Y; };
    // 특정 노드 진입 시 체크해야 할 규칙
    struct ConflictRule {
        int other_id;
        const Path* other_path_ptr;
    };
    struct SegmentRule {
        const Path* my_segment_ptr;           // 내가 현재 있는 경로
        std::vector<ConflictRule> conflicts; // 해당 구간에서 주의해야 할 타 차량들
    };

    // 현재 내 노드/구간에 따른 규칙 리스트
    struct Point { double x; double y; };

    // --- 유틸리티 함수 ---
        // 생성자 내부 또는 별도 초기화 함수
    void setup_conflict_rules() {
        my_active_rules_.clear();

        if (domain_id_ == 1) {
            my_active_rules_.push_back({&path21_51, {{3, &path22_25}}});
            my_active_rules_.push_back({&path51_46, {{2, &path60_52}, {2, &path49_55}, {3, &path48_58}}});
            my_active_rules_.push_back({&path9_56,  {{4, &path8_11}}});
            my_active_rules_.push_back({&path56_59, {{2, &path49_55}, {2, &path60_52}, {4, &path61_47}}});
        }
        else if (domain_id_ == 2) {
            my_active_rules_.push_back({&path60_52, {{1, &path56_59}, {1, &path51_46}, {3, &path48_58}}});
            my_active_rules_.push_back({&path52_24, {{3, &path22_25}}});
            my_active_rules_.push_back({&path49_55, {{1, &path51_46}, {1, &path56_59}, {4, &path61_47}}});
            my_active_rules_.push_back({&path55_12, {{4, &path8_11}}});
        }
        else if (domain_id_ == 3) {
            my_active_rules_.push_back({&path22_25, {{1, &path21_51}, {2, &path52_24}}});
            my_active_rules_.push_back({&path48_58, {{1, &path51_46}, {2, &path60_52}}});
        }
        else if (domain_id_ == 4) {
            my_active_rules_.push_back({&path61_47, {{1, &path56_59}, {2, &path49_55}}});
            my_active_rules_.push_back({&path8_11,  {{1, &path9_56}, {2, &path55_12}}});
        }
    }

    // 특정 좌표가 해당 경로 위에 있는지 확인 (임계값 0.2m)
    bool is_on_path(const Path& path, double x, double y, double threshold = 0.17) {
        if (path.X.empty()) return false;
        double min_d = 1e9;
        for (size_t i = 0; i < path.X.size(); ++i) {
            min_d = std::min(min_d, std::hypot(path.X[i] - x, path.Y[i] - y));
        }
        return min_d < threshold;
    }

    int find_closest_index(const Path& path, double x, double y) {
        int idx = 0; double min_d = 1e9;
        for (size_t i = 0; i < path.X.size(); ++i) {
            double d = std::hypot(path.X[i] - x, path.Y[i] - y);
            if (d < min_d) { min_d = d; idx = i; }
        }
        return idx;
    }

    // 하드코딩된 충돌 검사 (특정 타겟만 수행)
    bool check_hardcoded_collision(int other_id, const Path& other_path) {
        if (!others_received_[other_id]) return false;

        if (!is_on_path(other_path, others_pos_[other_id].x, others_pos_[other_id].y)) return false;

        int other_idx = find_closest_index(other_path, others_pos_[other_id].x, others_pos_[other_id].y);
        double my_accum = 0.0;
        double deadzone = 0.15; // 15cm 이내 차이는 "동시 진입"으로 간주

        for (int i = my_idx_; i < (int)my_path_.X.size() - 1 && my_accum < 1.0; ++i) {
            my_accum += std::hypot(my_path_.X[i+1]-my_path_.X[i], my_path_.Y[i+1]-my_path_.Y[i]);
            double other_accum = 0.0;

            for (int j = other_idx; j < (int)other_path.X.size() - 1 && other_accum < 1.5; ++j) {
                other_accum += std::hypot(other_path.X[j+1]-other_path.X[j], other_path.Y[j+1]-other_path.Y[j]);

                if (std::hypot(my_path_.X[i] - other_path.X[j], my_path_.Y[i] - other_path.Y[j]) < 0.25) {
                    // 1. 거리 차이가 deadzone보다 클 때: 더 먼 차가 양보
                    if (my_accum > other_accum + deadzone) {
                        return true;
                    }
                    // 2. 거리 차이가 거의 없을 때 (동시 진입): ID가 큰 차가 양보
                    else if (std::abs(my_accum - other_accum) <= deadzone) {
                        if (domain_id_ > (size_t)other_id) {
                            return true; // 내 ID가 크면 내가 멈춤
                        }
                    }
                }
            }
        }
        return false;
    }

    bool check_hv_collision_path_based() {
        // 감지 대상 HV 목록 (19, 20)
        std::vector<int> target_hvs = {19, 20};

        for (int id : target_hvs) {
            if (!others_received_[id] || others_paths_.find(id) == others_paths_.end()) continue;

            const auto& other_path = others_paths_[id];
            // 1. HV의 현재 경로상 인덱스 찾기
            int other_idx = find_closest_index(other_path, others_pos_[id].x, others_pos_[id].y);

            double my_accum_dist = 0.0;
            // 2. 내 경로 전방 탐색 (약 2.0m)
            for (int i = my_idx_; i < (int)my_path_.X.size() - 1; ++i) {
                double step_ego = std::hypot(my_path_.X[i+1] - my_path_.X[i], my_path_.Y[i+1] - my_path_.Y[i]);
                my_accum_dist += step_ego;
                if (my_accum_dist > 1.0) break; // 탐색 거리 제한

                double other_accum_dist = 0.0;
                // 3. HV 경로 전방 탐색 (약 2.0m)
                for (int j = other_idx; j < (int)other_path.X.size() - 1; ++j) {
                    double step_other = std::hypot(other_path.X[j+1] - other_path.X[j], other_path.Y[j+1] - other_path.Y[j]);
                    other_accum_dist += step_other;
                    if (other_accum_dist > 1.5) break;
                    // 4. 두 경로 점 사이의 거리 계산 (충돌 지점 확인)
                    double gap = std::hypot(my_path_.X[i] - other_path.X[j], my_path_.Y[i] - other_path.Y[j]);

                    // 5. 충돌 위험 판단 (임계값 0.25m)
                    if (gap < 0.5) {
                        // [핵심 로직]
                        // HV가 충돌 지점까지 남은 거리(other_accum_dist)가
                        // 내가 충돌 지점까지 남은 거리(my_accum_dist)보다 가깝거나 비슷하다면 정지
                        // double safety_margin = 0.3; // HV에게 양보할 거리 마진
                        if (other_accum_dist < my_accum_dist) {
                            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "HV_%d Path Collision! Ego dist: %.2f, HV dist: %.2f", id, my_accum_dist, other_accum_dist);
                            return true;
                        }
                    }
                }
            }
        }
        return false;
    }
    CollisionInfo check_collision_optimized() {
        CollisionInfo info;
        if (my_path_.X.empty()) return info;

        // 내 현재 인덱스 업데이트
        my_idx_ = find_closest_index(my_path_, curr_x_, curr_y_);

        for (auto const& [id, other_path] : others_paths_) {
            if (!others_received_[id]) continue;

            // [핵심] 타 차량의 인덱스를 직접 계산 (통신 지연 보정)
            int other_idx = find_closest_index(other_path, others_pos_[id].x, others_pos_[id].y);

            //todo: ego_pose 상대 pose 거리계산해서 3.0이상이면 건너뛰기
            if (id < 19) continue;

            double my_accum_dist = 0.0;
            // 1. 내 경로 전방 2.0m 탐색
            for (int i = my_idx_; i < (int)my_path_.X.size() - 1; ++i) {
                my_accum_dist += std::hypot(my_path_.X[i+1] - my_path_.X[i], my_path_.Y[i+1] - my_path_.Y[i]);
                if (my_accum_dist > 1.0) break;

                // 2. 상대 경로 탐색 (현재 위치의 5인덱스 뒤부터 검사하여 '꼬리 충돌' 방지)
                int start_j = std::max(0, other_idx - 5);
                double other_accum_dist = (start_j < other_idx) ? -0.1 : 0.0;

                for (int j = start_j; j < (int)other_path.X.size() - 1; ++j) {
                    // 상대방도 현재 위치 기준 약 2.0m 앞까지만 확인
                    double step = std::hypot(other_path.X[j+1] - other_path.X[j], other_path.Y[j+1] - other_path.Y[j]);
                    other_accum_dist += step;
                    double dist;
                    if (id >= 19)
                        dist = 2.0;
                    else
                        dist = 1.0;
                    if (other_accum_dist > dist) break;

                    double gap = std::hypot(my_path_.X[i] - other_path.X[j], my_path_.Y[i] - other_path.Y[j]);

                    // 3. 충돌 임계값 (소형차 기준 0.25m)
                    double collision_threshold = 0.17;
                    if (id >= 19)
                        collision_threshold = 0.5;
                    if (gap < collision_threshold) {
                        if (my_accum_dist < info.dist_to_collision_ego) {
                            info.collision_predicted = true;
                            info.dist_to_collision_ego = my_accum_dist;
                            info.dist_to_collision_other = std::max(0.0, other_accum_dist);
                            info.other_id = id;
                        }
                        goto next_vehicle;
                    }
                }
            }
            next_vehicle:;
        }
        return info;
    }
    void control_loop() {
        if (!pose_received_) return;
        my_idx_ = find_closest_index(my_path_, curr_x_, curr_y_);
        double target_v = v_;

        // 1. 미리 정의된 내 규칙 리스트 순회
        for (const auto& segment : my_active_rules_) {
            // 내가 현재 이 구간(Path) 위에 있는지 확인
            if (is_on_path(*(segment.my_segment_ptr), curr_x_, curr_y_)) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                    "[Segment Focus] On segment, checking conflicts");
                // 해당 구간의 모든 갈등 상황 체크
                for (const auto& conflict : segment.conflicts) {
                    if (check_hardcoded_collision(conflict.other_id, *(conflict.other_path_ptr))) {
                        target_v = 0.5;
                        if (conflict.other_id < domain_id_)
                            target_v = 0.0;
                        if (domain_id_ == 2&& is_on_path(path60_52_exit, curr_x_, curr_y_))
                            target_v = 2.0;
                        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                            "[Segment Focus] Yielding to CAV%d at conflict point", conflict.other_id);
                        break;
                    }
                }
            }
            else {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                    "[Segment Focus] Not on segment, skipping checks");
            }
            if (target_v == 0.0) break; // 이미 멈췄으면 다른 구간 검사 중단
        }

        //     // HV(19, 20)는 항상 감시 (교차로 Circle 내에서만)
        // if (target_v > 0.01 && check_hv_collision_circle()) {
        //     target_v = 0.0;
        // }


        // if (!pose_received_) return;

        // 1. 기본 충돌 검사 (CAV 간)
        CollisionInfo col = check_collision_optimized();

        // 2. [우선순위 1] HV(19, 20) 차량과의 충돌 검사
        // 교차로(Circle) 진입 여부와 상관없이 HV는 위험하므로 먼저 체크합니다.
        if (check_hv_collision_path_based()) {
            target_v = 0.0;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, "Stopping for HV (Path-based)");
        }
        // 3. [우선순위 2] 다른 CAV와의 충돌 회피 (Yield 로직)
        else if (col.collision_predicted) {
            double dist_diff = col.dist_to_collision_ego - col.dist_to_collision_other;
            double deadzone = 0.15; // 2cm(0.02)는 너무 작으므로 15cm 정도로 조정 권장

            // 내가 더 멀리 있거나, 거리가 비슷할 때 내 ID가 더 크면 양보
            if (dist_diff > deadzone) {
                target_v = 0.0;
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Yielding: I am farther than CAV%d", col.other_id);
            }
            else if (std::abs(dist_diff) <= deadzone) {
                if (domain_id_ > (size_t)col.other_id) { // 내 ID가 크면 양보
                    target_v = 0.0;
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Yielding: Lower ID (CAV%d) has priority", col.other_id);
                }
            }
        }

                // 4. [우선순위 3] 교차로(Circle) 특수 속도 적용
        // 정지 상황이 아닐 때만 속도를 2.0으로 올립니다.
        if (target_v > 0.01) {
            int circle_idx = find_closest_index(circle_path_, curr_x_, curr_y_);
            double circle_dist = std::hypot(circle_path_.X[circle_idx] - curr_x_, circle_path_.Y[circle_idx] - curr_y_);

            if (circle_dist < 0.5) {
                target_v = 2.0;
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "In Circle: Speeding up to 2.0m/s");
            }
        }


        // 제어량 발행
        double curvature = compute_curvature();
        geometry_msgs::msg::Accel msg;
        msg.linear.x = target_v;
        msg.angular.z = std::clamp(target_v * curvature, -M_PI, M_PI);
        accel_pub_->publish(msg);
    }

    bool check_hv_collision_circle() {
        int circle_idx = find_closest_index(circle_path_, curr_x_, curr_y_);
        if (std::hypot(circle_path_.X[circle_idx]-curr_x_, circle_path_.Y[circle_idx]-curr_y_) > 0.5) return false;

        for (int id : {19, 20}) {
            if (!others_received_[id]) continue;
            double dist = std::hypot(curr_x_ - others_pos_[id].x, curr_y_ - others_pos_[id].y);
            if (dist < 0.7) return true;
        }
        return false;
    }

    // --- 나머지 초기화 및 커버처 함수 ---

    bool init_config_and_paths() {
        std::string pkg_path = ament_index_cpp::get_package_share_directory("pure_pursuit_controller");
        try {
            YAML::Node config = YAML::LoadFile(pkg_path + "/config/cav_params.yaml");
            std::string k = "cav" + std::to_string(domain_id_);
            v_ = config["problem_three"][k]["init_speed"].as<double>(1.0);
            ld_ = config["problem_three"][k]["lookahead"].as<double>(0.5);
            my_path_ = load_path_json(pkg_path + "/config/" + config["problem_three"][k]["path"].as<std::string>());

            for (int i = 1; i <= 4; ++i) {
                if (i == (int)domain_id_) continue;
                std::string other_key = "cav" + std::to_string(i);
                if (config["problem_three"][other_key]) {
                    others_paths_[i] = load_path_json(pkg_path + "/config/" + config["problem_three"][other_key]["path"].as<std::string>());
                }
            }

            others_paths_[19] = load_path_json(pkg_path + "/config/roundabout_lane_two.json");
            others_paths_[20] = load_path_json(pkg_path + "/config/roundabout_lane_two.json");
            circle_path_ = load_path_json(pkg_path + "/config/circle.json");
            // 모든 세그먼트 경로 로드
            path8_11 = load_path_json(pkg_path + "/config/8_11_fixed.json"); //아래 직선 좌측이동
            path9_56 = load_path_json(pkg_path + "/config/9_56.json");
            path21_51 = load_path_json(pkg_path + "/config/21_51.json");
            path22_25 = load_path_json(pkg_path + "/config/22_25_fixed.json"); //위에 직선 우측이동
            path48_58 = load_path_json(pkg_path + "/config/48_58_fixed.json"); // 중간 직선 좌측이동
            path49_55 = load_path_json(pkg_path + "/config/49_55_fixed.json");
            path51_46 = load_path_json(pkg_path + "/config/21_51_46.json");
            path52_24 = load_path_json(pkg_path + "/config/52_24.json");
            path55_12 = load_path_json(pkg_path + "/config/55_12.json");
            path56_59 = load_path_json(pkg_path + "/config/56_59.json"); //
            path60_52 = load_path_json(pkg_path + "/config/60_52.json");
            path61_47 = load_path_json(pkg_path + "/config/61_47_fixed.json");// 중간직선 우측이동
            path60_52_exit = load_path_json(pkg_path + "/config/60_52_fixed.json");// 중간직선 우측이동
            circle_path_ = load_path_json(pkg_path + "/config/circle.json");

            return true;
        } catch (...) { return false; }
    }

    Path load_path_json(const std::string& path) {
        Path p; std::ifstream f(path);
        if (f.is_open()) { json j; f >> j; p.X = j["X"].get<std::vector<double>>(); p.Y = j["Y"].get<std::vector<double>>(); }
        return p;
    }

    double compute_curvature() {
        int target_idx = -1;
        for (size_t i = my_idx_; i < my_path_.X.size(); ++i) {
            if (std::hypot(my_path_.X[i] - curr_x_, my_path_.Y[i] - curr_y_) >= ld_) { target_idx = i; break; }
        }
        if (target_idx == -1) target_idx = my_path_.X.size() - 1;
        double local_y = -std::sin(curr_yaw_) * (my_path_.X[target_idx] - curr_x_) + std::cos(curr_yaw_) * (my_path_.Y[target_idx] - curr_y_);
        return (2.0 * local_y) / (ld_ * ld_ + 1e-9);
    }

    size_t domain_id_;
    int my_idx_ = 0;
    double curr_x_, curr_y_, curr_yaw_, v_, ld_;
    bool pose_received_ = false;
    Path my_path_, circle_path_, path8_11, path9_56, path21_51, path22_25, path48_58, path49_55, path51_46, path52_24, path55_12, path56_59, path60_52, path61_47;
    Path path60_52_exit;
    std::map<int, Point> others_pos_;
    std::map<int, bool> others_received_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    std::map<int, rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> other_subs_;
    rclcpp::Publisher<geometry_msgs::msg::Accel>::SharedPtr accel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<SegmentRule> my_active_rules_;


    std::map<int, Path> others_paths_;

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuitNode>());
    rclcpp::shutdown();
    return 0;
}
