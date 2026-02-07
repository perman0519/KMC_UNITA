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

class Quiz2 : public rclcpp::Node {
public:
    Quiz2() : Node("quiz2_node") {
        if (!init_config() || !init_all_paths()) {
            RCLCPP_ERROR(this->get_logger(), "초기화 실패");
            return;
        }
        else {
            RCLCPP_INFO(this->get_logger(), "초기화 성공");
            RCLCPP_INFO(this->get_logger(), "v=%.2f, ld=%.2f", v_, ld_);
            RCLCPP_INFO(this->get_logger(), "모든 경로 로드 완료");
            RCLCPP_INFO(this->get_logger(), "1차선 경로 점 수: %zu", lanes_path_[1].X.size());
            RCLCPP_INFO(this->get_logger(), "2차선 경로 점 수: %zu", lanes_path_[2].X.size());
            RCLCPP_INFO(this->get_logger(), "3차선 경로 점 수: %zu", lanes_path_[3].X.size());
        }
        auto qos = rclcpp::SensorDataQoS();

        // 1. Ego 위치 구독
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/Ego_pose", qos, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                this->curr_x_ = msg->pose.position.x;
                this->curr_y_ = msg->pose.position.y;

                // ✅ 너가 확인한대로 z가 yaw(rad)인 환경 유지
                this->curr_yaw_ = msg->pose.orientation.z;

                // ✅ 초기 안정화를 위해 pose 프레임 카운트
                pose_received_ = true;
                pose_count_++;
            });

        // 19~22번 (Line 3 HV)
        for (int i = 19; i <= 22; ++i) {
            std::string topic = "/HV_" + std::to_string(i);
            line3_pose_sub_[i] = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                topic, qos, [this, i](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                    this->line3_hvs_x_[i] = msg->pose.position.x;
                    this->line3_hvs_y_[i] = msg->pose.position.y;
                });
        }
        // 23~30번 (Line 2 HV)
        for (int i = 23; i <= 30; ++i) {
            std::string topic = "/HV_" + std::to_string(i);
            line2_pose_sub_[i] = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                topic, qos, [this, i](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                    this->line2_hvs_x_[i] = msg->pose.position.x;
                    this->line2_hvs_y_[i] = msg->pose.position.y;
                });
        }
        // 31~36번 (Line 1 HV)
        for (int i = 31; i <= 36; ++i) {
            std::string topic = "/HV_" + std::to_string(i);
            line1_pose_sub_[i] = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                topic, qos, [this, i](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                    this->line1_hvs_x_[i] = msg->pose.position.x;
                    this->line1_hvs_y_[i] = msg->pose.position.y;
                });
        }

        accel_pub_ = this->create_publisher<geometry_msgs::msg::Accel>("/Accel", 10);
        timer_ = this->create_wall_timer(50ms, std::bind(&Quiz2::control_loop, this));

        current_lane_ = 2;
    }

private:
    struct Path { std::vector<double> X, Y; };

    bool init_config() {
        std::string pkg_path = ament_index_cpp::get_package_share_directory("pure_pursuit_controller");
        try {
            YAML::Node config = YAML::LoadFile(pkg_path + "/config/cav_params.yaml");
            if (config["problem_two"] && config["problem_two"]["cav1"]) {
                auto p2 = config["problem_two"]["cav1"];
                v_ = p2["init_speed"].as<double>(1.0);
                ld_ = p2["lookahead"].as<double>(0.5);
                return true;
            }
        } catch (...) {}
        v_ = 1.0; ld_ = 0.5;
        return true;
    }

    bool init_all_paths() {
        std::string pkg_path = ament_index_cpp::get_package_share_directory("pure_pursuit_controller");
        lanes_path_[1] = load_path_json(pkg_path + "/config/highway_line1.json");
        lanes_path_[2] = load_path_json(pkg_path + "/config/highway_line2.json");
        lanes_path_[3] = load_path_json(pkg_path + "/config/highway_line3.json");

        lane_27_30_1_3_6_path_ = load_path_json(pkg_path + "/config/27_30_1_3_6.json");
        lane_28_31_1_3_5_path_front = load_path_json(pkg_path + "/config/28_31_1_3_5_front.json");
        lane_28_31_1_3_5_path_cross = load_path_json(pkg_path + "/config/28_31_1_3_5_cross.json");
        lane_28_31_1_3_5_path_back = load_path_json(pkg_path + "/config/28_31_1_3_5_back.json");
        lane_28_31_1_3_5_path_change = load_path_json(pkg_path + "/config/28_31_1_3_5_change.json");
        lane_28_31_1_3_5_path_lane2 = load_path_json(pkg_path + "/config/28_31_1_3_5_lane2.json");

        return (!lanes_path_[1].X.empty() && !lanes_path_[2].X.empty() && !lanes_path_[3].X.empty());
    }

    Path load_path_json(const std::string& path) {
        Path p;
        std::ifstream f(path);
        if (f.is_open()) {
            json j; f >> j;
            p.X = j["X"].get<std::vector<double>>();
            p.Y = j["Y"].get<std::vector<double>>();
        }
        return p;
    }

    // ✅ 현재 위치가 해당 path에서 얼마나 떨어져 있는지 (초기 튐 방지에 사용)
    double min_dist_to_path(const Path& path) {
        if (path.X.empty()) return 1e9;
        double min_dist = 1e9;
        for (size_t i = 0; i < path.X.size(); ++i) {
            double d = std::hypot(path.X[i] - curr_x_, path.Y[i] - curr_y_);
            if (d < min_dist) min_dist = d;
        }
        return min_dist;
    }

    bool is_on_path(const Path& path, double threshold) {
        return (min_dist_to_path(path) < threshold);
    }

    double compute_curvature(const Path& path) {
        int closest_idx = 0;
        double min_d = 1e9;
        for (size_t i = 0; i < path.X.size(); ++i) {
            double d = std::hypot(path.X[i] - curr_x_, path.Y[i] - curr_y_);
            if (d < min_d) { min_d = d; closest_idx = (int)i; }
        }

        int target_idx = -1;
        for (size_t i = closest_idx; i < path.X.size(); ++i) {
            double d = std::hypot(path.X[i] - curr_x_, path.Y[i] - curr_y_);
            if (d >= ld_) { target_idx = (int)i; break; }
        }
        if (target_idx == -1) target_idx = (int)path.X.size() - 1;

        double dx = path.X[target_idx] - curr_x_;
        double dy = path.Y[target_idx] - curr_y_;

        double local_y = -std::sin(curr_yaw_) * dx + std::cos(curr_yaw_) * dy;
        return (2.0 * local_y) / (ld_ * ld_ + 1e-9);
    }

    bool is_lane_blocked(int lane_num) {
        const auto& path = lanes_path_[lane_num];
        std::vector<std::map<int, double>*> target_x_maps;
        std::vector<std::map<int, double>*> target_y_maps;

        int closest_idx = 0; double min_d = 1e9;
        for(size_t i=0; i < path.X.size(); ++i) {
            double d = std::hypot(path.X[i] - curr_x_, path.Y[i] - curr_y_);
            if(d < min_d) { min_d = d; closest_idx = (int)i; }
        }

        if (lane_num == 1) {
            target_x_maps.push_back(&line1_hvs_x_); target_y_maps.push_back(&line1_hvs_y_);
        } else if (lane_num == 2) {
            target_x_maps.push_back(&line2_hvs_x_); target_y_maps.push_back(&line2_hvs_y_);
        } else if (lane_num == 3) {
            target_x_maps.push_back(&line3_hvs_x_); target_y_maps.push_back(&line3_hvs_y_);
            if (is_on_path(lane_28_31_1_3_5_path_cross, 0.2)) {
                target_x_maps.push_back(&line2_hvs_x_); target_y_maps.push_back(&line2_hvs_y_);
            }
        }

        int search_steps = 110;
        for (int j = closest_idx; j < closest_idx + search_steps; ++j) {
            int idx = j % (int)path.X.size();
            for (size_t m = 0; m < target_x_maps.size(); ++m) {
                auto* x_map = target_x_maps[m];
                auto* y_map = target_y_maps[m];
                for (auto const& kv : *x_map) {
                    int id = kv.first;
                    double h_x = kv.second;
                    double h_y = (*y_map)[id];
                    double dist = std::hypot(h_x - path.X[idx], h_y - path.Y[idx]);
                    if (dist < 0.17) return true;
                }
            }
        }
        return false;
    }

    bool is_lane_behind_blocked(int lane_num) {
        const auto& path = lanes_path_[lane_num];
        std::vector<std::map<int, double>*> target_x_maps;
        std::vector<std::map<int, double>*> target_y_maps;

        int closest_idx = 0; double min_d = 1e9;
        for(size_t i=0; i < path.X.size(); ++i) {
            double d = std::hypot(path.X[i] - curr_x_, path.Y[i] - curr_y_);
            if(d < min_d) { min_d = d; closest_idx = (int)i; }
        }

        if (lane_num == 1) {
            target_x_maps.push_back(&line1_hvs_x_); target_y_maps.push_back(&line1_hvs_y_);
        } else if (lane_num == 2) {
            target_x_maps.push_back(&line2_hvs_x_); target_y_maps.push_back(&line2_hvs_y_);
        } else if (lane_num == 3) {
            target_x_maps.push_back(&line3_hvs_x_); target_y_maps.push_back(&line3_hvs_y_);
            if (is_on_path(lane_28_31_1_3_5_path_cross, 0.2)) {
                target_x_maps.push_back(&line2_hvs_x_); target_y_maps.push_back(&line2_hvs_y_);
            }
        }

        int search_steps = 50;
        closest_idx -= 50;
        if (closest_idx < 0) closest_idx = 0;

        for (int j = closest_idx; j < closest_idx + search_steps; ++j) {
            int idx = j % (int)path.X.size();
            for (size_t m = 0; m < target_x_maps.size(); ++m) {
                auto* x_map = target_x_maps[m];
                auto* y_map = target_y_maps[m];
                for (auto const& kv : *x_map) {
                    int id = kv.first;
                    double h_x = kv.second;
                    double h_y = (*y_map)[id];
                    double dist = std::hypot(h_x - path.X[idx], h_y - path.Y[idx]);
                    if (dist < 0.17) return true;
                }
            }
        }
        return false;
    }
void control_loop() {
        // 1. 초기 상태 체크 (Pose 수신 확인 및 안정화 대기)
        if (!pose_received_) return;
        if (pose_count_ < pose_start_threshold_) {
            publish_cmd(0.0, 0.0);
            return;
        }

        // 2. 경로 이탈 방지 (현재 차선과의 거리가 너무 멀면 정지)
        double dist_to_lane = min_dist_to_path(lanes_path_[current_lane_]);
        if (dist_to_lane > path_lock_dist_) {
            publish_cmd(0.0, 0.0);
            return;
        }

        double target_v = v_;

        // 3. 특수 구간(JSON 경로) 판단
        bool on_forbidden_lane3_path = is_on_path(lane_27_30_1_3_6_path_, 0.1);
        bool on_force_lane2_path_back = is_on_path(lane_28_31_1_3_5_path_back, 0.1);
        bool on_force_lane2_path_change = is_on_path(lane_28_31_1_3_5_path_change, 0.1);

        // 28~31번 합류 금지 구간 여부 (front 경로 활용)
        bool in_28_31_zone = is_on_path(lane_28_31_1_3_5_path_front, 0.5);

        // 4. 강제 2차선 복귀 판단 (특정 경로 위인 경우)
        if (on_force_lane2_path_back && current_lane_ != 2) {
            RCLCPP_INFO(this->get_logger(), "강제 복귀 경로 진입: 2차선으로 이동");
            current_lane_ = 2;
        }

        // 5. 장애물 감지 및 차선 변경 로직
        if (is_lane_blocked(current_lane_)) {
            if (current_lane_ == 2) {
                // [2차선 주행 중 장애물 발생]
                if (in_28_31_zone) {
                    // 28~31 구간(33번 부근)에서는 절대 차선을 바꾸지 않고 정지
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "합류 금지 구간: 대기 중");
                    target_v = 0.0;
                }
                else {
                    // 일반 구간: 1차선 또는 3차선으로 회피 시도
                    if (!on_forbidden_lane3_path && !is_lane_blocked(3)) {
                        RCLCPP_INFO(this->get_logger(), "2 -> 3 차선 변경");
                        current_lane_ = 3;
                    }
                    else if (!is_lane_blocked(1)) {
                        RCLCPP_INFO(this->get_logger(), "2 -> 1 차선 변경");
                        current_lane_ = 1;
                    }
                    else {
                        // 갈 수 있는 차선이 모두 막힘
                        target_v = 0.0;
                    }
                }
            }
            else {
                // [1차선 또는 3차선 주행 중 장애물 발생]
                // 1->3 또는 3->1 점프 금지: 오직 2차선 복귀 가능성만 체크
                if (!is_lane_blocked(2) && !is_lane_behind_blocked(2)) {
                    RCLCPP_INFO(this->get_logger(), "%d -> 2 차선 복귀", current_lane_);
                    current_lane_ = 2;
                } else {
                    // 2차선이 막혀서 복귀 불가능하면 정지
                    target_v = 0.0;
                }
            }
        }

        // 6. 특수 합류 로직 (합류 지점에서 일시 정지 후 2차선 진입)
        if (on_force_lane2_path_change && current_lane_ == 3) {
            RCLCPP_INFO(this->get_logger(), "합류 지점: 2차선 변경 및 일시 정지");
            current_lane_ = 2;
            target_v = 0.0;
        }

        // 7. 후방 차량 가속 로직 (필요 시)
        if (is_lane_behind_blocked(current_lane_)) {
            target_v = 2.0;
        }

        // 8. 제어값 계산 및 명령 발행
        double curvature = compute_curvature(lanes_path_[current_lane_]);
        double omega = target_v * curvature;

        // 초기 튐 방지 및 과도한 조향 방지 (Clamp)
        omega = std::clamp(omega, -omega_max_, omega_max_);

        publish_cmd(target_v, omega);
    }

    void publish_cmd(double v, double omega) {
        geometry_msgs::msg::Accel msg;
        msg.linear.x = v;
        msg.angular.z = omega;   // 원본 유지
        accel_pub_->publish(msg);
    }

    // ===== 멤버 =====
    double curr_x_=0, curr_y_=0, curr_yaw_=0, v_=1.0, ld_=0.3;
    int current_lane_ = 2;

    std::map<int, Path> lanes_path_;
    std::map<int, double> line1_hvs_x_, line1_hvs_y_;
    std::map<int, double> line2_hvs_x_, line2_hvs_y_;
    std::map<int, double> line3_hvs_x_, line3_hvs_y_;

    Path lane_27_30_1_3_6_path_;
    Path lane_28_31_1_3_5_path_front;
    Path lane_28_31_1_3_5_path_back;
    Path lane_28_31_1_3_5_path_cross;
    Path lane_28_31_1_3_5_path_change;
    Path lane_28_31_1_3_5_path_lane2;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    std::map<int, rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> line1_pose_sub_;
    std::map<int, rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> line2_pose_sub_;
    std::map<int, rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> line3_pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Accel>::SharedPtr accel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // ✅ 초기 문제만 잡기 위한 최소 변수
    bool pose_received_ = false;
    int pose_count_ = 0;
    int pose_start_threshold_ = 5;   // 10프레임(=약 1초 내외) 받고 시작

    double omega_max_ = 3.1;          // 네 “잘 되는 코드”와 동일
    double path_lock_dist_ = 0.8;     // 경로에서 0.8m 이상 멀면 제어 시작 안 함
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Quiz2>());
    rclcpp::shutdown();
    return 0;
}
