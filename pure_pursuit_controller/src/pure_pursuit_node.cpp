#include <memory>
#include <string>
#include <fstream>
#include <vector>
#include <cmath>
#include <algorithm>
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
        this->declare_parameter("problem_name", "problem_one_one");
        this->declare_parameter("lane_name", "lane_two");
        this->declare_parameter("vehicle_id", "01");

        std::string prob = this->get_parameter("problem_name").as_string();
        std::string lane = this->get_parameter("lane_name").as_string();
        std::string vid  = this->get_parameter("vehicle_id").as_string();

        // load_yaml_config(prob, lane);
        if (!init_path()) {
            RCLCPP_ERROR(this->get_logger(), "경로 파일을 로드하지 못했습니다.");
            return;
        }

        // 2. QoS 설정 (시뮬레이터 사양: SensorDataQoS)
        auto qos_profile = rclcpp::SensorDataQoS();

        // 3. 구독자 생성: /Ego_pose
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/Ego_pose", qos_profile,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                this->curr_x_ = msg->pose.position.x;
                this->curr_y_ = msg->pose.position.y;
                this->curr_yaw_ = msg->pose.orientation.z; // 시뮬레이터 특이사항: z가 yaw(rad)
            });

        other_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/CAV" + other_cav_id_ + "_pose", qos_profile, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            other_x_ = msg->pose.position.x;
            other_y_ = msg->pose.position.y;
            other_yaw_ = msg->pose.orientation.z;
        });

        other_accel_sub_ = this->create_subscription<geometry_msgs::msg::Accel>(
            "/CAV" + other_cav_id_ + "_accel", qos_profile, [this](const geometry_msgs::msg::Accel::SharedPtr msg) {
            other_v_ = msg->linear.x;
            other_omega_ = msg->angular.z;
        });

        // 4. 발행자 생성: /Accel
        accel_pub_ = this->create_publisher<geometry_msgs::msg::Accel>("/Accel", 10);

        // 5. 제어 루프 타이머 (10Hz)
        timer_ = this->create_wall_timer(100ms, std::bind(&PurePursuitNode::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "Pure Pursuit Node가 성공적으로 시작되었습니다.");
    }

private:

    struct Point { double x; double y; };

    // 특정 경로에서 현재 위치 기준 Look-ahead target point를 찾아주는 헬퍼 함수
    Point get_target_point(const std::vector<double>& px, const std::vector<double>& py,
                           double cur_x, double cur_y, double ld) {
        if (px.empty()) return {cur_x, cur_y};

        int closest_idx = 0;
        double min_dist = 1e9;

        // 1. 현재 위치에서 가장 가까운 인덱스 찾기
        for (size_t i = 0; i < px.size(); ++i) {
            double d = std::hypot(px[i] - cur_x, py[i] - cur_y);
            if (d < min_dist) {
                min_dist = d;
                closest_idx = i;
            }
        }

        // 2. Ld 거리만큼 떨어진 타겟 인덱스 찾기
        int target_idx = closest_idx;
        for (size_t i = closest_idx; i < px.size(); ++i) {
            if (std::hypot(px[i] - cur_x, py[i] - cur_y) >= ld) {
                target_idx = i;
                break;
            }
        }
        return {px[target_idx], py[target_idx]};
    }

    // JSON 경로 로드 함수
    bool init_path() {
        std::string pkg_path = ament_index_cpp::get_package_share_directory("pure_pursuit_controller");
        try {
            std::string yaml_path = pkg_path + "/config/cav_params.yaml";
            YAML::Node config;
            try{
                config = YAML::LoadFile(yaml_path);
            } catch (const std::exception& e) {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Failed to load YAML (%s), using defaults",
                    e.what()
                );
            }
            const char* env_domain = std::getenv("ROS_DOMAIN_ID");
            domain_id_ = (env_domain != nullptr) ? std::atoi(env_domain) : 0;
            std::string json_path = "";
            std::string other_json_path = "";
            if (config && config["problem_one"]) {
                if (domain_id_ == 1 && config["problem_one"]["cav1"]) {
                    auto p1 = config["problem_one"]["cav1"];
                    v_ = p1["init_speed"].as<double>(v_);
                    ld_ = p1["lookahead"].as<double>(ld_);
                    json_path = p1["path"].as<std::string>("");
                    other_json_path = config["problem_one"]["cav2"]["path"].as<std::string>("");
                    other_cav_id_ = "2";
                } else if (domain_id_ == 2 && config["problem_one"]["cav2"]) {
                    auto p1 = config["problem_one"]["cav2"];
                    v_ = p1["init_speed"].as<double>(v_);
                    ld_ = p1["lookahead"].as<double>(ld_);
                    json_path = p1["path"].as<std::string>("");
                    other_json_path = config["problem_one"]["cav1"]["path"].as<std::string>("");
                    other_cav_id_ = "1";
                }
            }
            auto new_json_path = pkg_path + "/config/" + json_path;
            std::ifstream f(new_json_path);
            if (!f.is_open()) return false;

            json j;
            f >> j;
            path_x_ = j["X"].get<std::vector<double>>();
            path_y_ = j["Y"].get<std::vector<double>>();

            auto other_new_json_path = pkg_path + "/config/" + other_json_path;
            std::ifstream of(other_new_json_path);
            if (!of.is_open()) return false;

            json oj;
            of >> oj;
            other_path_x_ = oj["X"].get<std::vector<double>>();
            other_path_y_ = oj["Y"].get<std::vector<double>>();

            RCLCPP_INFO(
                this->get_logger(),
                "path loaded: %s (points: %zu), v=%.2f, ld=%.2f",
                json_path.c_str(), path_x_.size(), v_, ld_
            );
            RCLCPP_INFO(
                this->get_logger(),
                "other path loaded: %s (points: %zu)",
                other_json_path.c_str(), other_path_x_.size()
            );
            return true;
        } catch (...) {
            return false;
        }
    }

    // [핵심 로직] 각속도(Omega) 계산 함수
    double compute_yaw_rate() {
        int target_idx = -1;
        double min_dist = std::numeric_limits<double>::max();

        // 1. 현재 위치에서 가장 가까운 경로 점 찾기
        int closest_idx = 0;
        for (size_t i = 0; i < path_x_.size(); ++i) {
            double d = std::sqrt(std::pow(path_x_[i] - curr_x_, 2) + std::pow(path_y_[i] - curr_y_, 2));
            if (d < min_dist) {
                min_dist = d;
                closest_idx = i;
            }
        }

        // 2. Ld(Look-ahead distance)만큼 떨어진 목표 점(target_idx) 찾기
        for (size_t i = closest_idx; i < path_x_.size(); ++i) {
            double d = std::sqrt(std::pow(path_x_[i] - curr_x_, 2) + std::pow(path_y_[i] - curr_y_, 2));
            if (d >= ld_) {
                target_idx = i;
                break;
            }
        }

        // 경로 끝에 도달했을 경우 마지막 점을 목표로 설정
        if (target_idx == -1) target_idx = path_x_.size() - 1;

        // 3. 목표 점을 차량 기준 로컬 좌표계로 변환
        double dx = path_x_[target_idx] - curr_x_;
        double dy = path_y_[target_idx] - curr_y_;

        // 로컬 좌표계에서의 y값 (차량 진행 방향 기준 좌우 편차)
        double local_y = -std::sin(curr_yaw_) * dx + std::cos(curr_yaw_) * dy;

        // 4. 곡률(Curvature) 계산: kappa = 2*y / Ld^2
        double curvature = (2.0 * local_y) / (ld_ * ld_);

        // 5. 각속도 결정: omega = v * kappa
        double omega = v_ * curvature;

        // 조향각 제한 (Saturation: 예시로 +-2.0 rad/s 제한)
        return std::clamp(omega, -M_PI, M_PI);
    }

    double compute_curvature() { // 이름 변경: yaw_rate -> curvature
    // ... (목표점 target_idx 찾는 로직은 동일) ...

        int target_idx = -1;
        double min_dist = std::numeric_limits<double>::max();

        // 1. 현재 위치에서 가장 가까운 경로 점 찾기
        int closest_idx = 0;
        for (size_t i = 0; i < path_x_.size(); ++i) {
            double d = std::sqrt(std::pow(path_x_[i] - curr_x_, 2) + std::pow(path_y_[i] - curr_y_, 2));
            if (d < min_dist) {
                min_dist = d;
                closest_idx = i;
            }
        }

        // 2. Ld(Look-ahead distance)만큼 떨어진 목표 점(target_idx) 찾기
        for (size_t i = closest_idx; i < path_x_.size(); ++i) {
            double d = std::sqrt(std::pow(path_x_[i] - curr_x_, 2) + std::pow(path_y_[i] - curr_y_, 2));
            if (d >= ld_) {
                target_idx = i;
                break;
            }
        }

        // 경로 끝에 도달했을 경우 마지막 점을 목표로 설정
        if (target_idx == -1) target_idx = path_x_.size() - 1;

        double dx = path_x_[target_idx] - curr_x_;
        double dy = path_y_[target_idx] - curr_y_;
        double local_y = -std::sin(curr_yaw_) * dx + std::cos(curr_yaw_) * dy;

        // 곡률 kappa = 2*y / Ld^2 (속도를 곱하지 않음!)
        double curvature = (2.0 * local_y) / (ld_ * ld_);
        return curvature;
    }

    void rk4_step(double &x, double &y, double &yaw, double v, double omega, double dt_=0.05)
    {
        auto f = [&](double Yaw, double &dx, double &dy, double &dyaw)
        {
            dx = v * std::cos(Yaw);
            dy = v * std::sin(Yaw);
            dyaw = omega;
        };

        double k1x, k1y, k1yaw;
        double k2x, k2y, k2yaw;
        double k3x, k3y, k3yaw;
        double k4x, k4y, k4yaw;

        f(yaw, k1x, k1y, k1yaw);
        f(yaw + 0.5*dt_*k1yaw, k2x, k2y, k2yaw);
        f(yaw + 0.5*dt_*k2yaw, k3x, k3y, k3yaw);
        f(yaw + dt_*k3yaw,     k4x, k4y, k4yaw);

        double w = dt_ / 6.0;

        x   += w * (k1x + 2*k2x + 2*k3x + k4x);
        y   += w * (k1y + 2*k2y + 2*k3y + k4y);
        yaw += w * (k1yaw + 2*k2yaw + 2*k3yaw + k4yaw);

        while (yaw >  M_PI) yaw -= 2*M_PI;
        while (yaw < -M_PI) yaw += 2*M_PI;
    }
    // 충돌 정보를 담기 위한 구조체
    struct CollisionInfo {
        bool collision_predicted = false;
        double dist_to_collision_ego = 1e9;
        double dist_to_collision_other = 1e9;
    };

    // 경로(px, py)에서 현재 위치(cur_x, cur_y)와 가장 가까운 점의 인덱스를 반환
    int find_closest_index(const std::vector<double>& px, const std::vector<double>& py,
                           double cur_x, double cur_y) {
        int closest_idx = 0;
        double min_dist = std::numeric_limits<double>::max();

        for (size_t i = 0; i < px.size(); ++i) {
            // 유클리드 거리 계산
            double dist = std::hypot(px[i] - cur_x, py[i] - cur_y);
            if (dist < min_dist) {
                min_dist = dist;
                closest_idx = i;
            }
        }
        return closest_idx;
    }

    CollisionInfo check_collision_detail() {
        CollisionInfo info;
        if (path_x_.empty() || other_path_x_.empty()) return info;

        // 1. Ego와 Other의 현재 인덱스 찾기 (기존과 동일)
        int my_idx = find_closest_index(path_x_, path_y_, curr_x_, curr_y_);
        int other_idx = find_closest_index(other_path_x_, other_path_y_, other_x_, other_y_);

        double my_accumulated_dist = 0.0;
        // 2. 내 경로 5m 탐색
        for (int i = my_idx; i < (int)path_x_.size() - 1; ++i) {
            my_accumulated_dist += std::hypot(path_x_[i+1] - path_x_[i], path_y_[i+1] - path_y_[i]);
            if (my_accumulated_dist > 1.5) break;

            double other_accumulated_dist = 0.0;
            // 3. 상대 경로 5m 탐색
            for (int j = other_idx; j < (int)other_path_x_.size() - 1; ++j) {
                other_accumulated_dist += std::hypot(other_path_x_[j+1] - other_path_x_[j], other_path_y_[j+1] - other_path_y_[j]);
                if (other_accumulated_dist > 1.5) break;

                // 4. 두 경로 점 사이의 거리 체크 (충돌 지점 발견)
                double gap = std::hypot(path_x_[i] - other_path_x_[j], path_y_[i] - other_path_y_[j]);
                if (gap < 0.2) { // 충돌 임계값
                    info.collision_predicted = true;
                    info.dist_to_collision_ego = my_accumulated_dist;
                    info.dist_to_collision_other = other_accumulated_dist;
                    return info; // 첫 번째 충돌 지점 정보 반환
                }
            }
        }
        return info;
    }

    void control_loop() {
        CollisionInfo col = check_collision_detail();
        double target_v = v_;

        if (col.collision_predicted) {
            // [핵심 로직] 우선순위 판단
            // 내가 상대보다 충돌 지점에 더 멀리 있을 때만 멈춤
            if (col.dist_to_collision_ego > col.dist_to_collision_other) {
                target_v = 0.0;
                RCLCPP_WARN(this->get_logger(), "Yielding! Other is closer to conflict point.");
            }
            // 거리가 아주 비슷해서 교착상태가 우려될 때만 domain_id 사용
            else if (std::abs(col.dist_to_collision_ego - col.dist_to_collision_other) < 0.1) {
                if (domain_id_ != 1) { // 1번이 아니면 양보
                    target_v = 0.0;
                }
            }
            // 그 외(내가 더 가깝거나 앞차인 경우)에는 target_v = v_ 유지하여 진행
        }

        // 곡률 계산 및 발행 (기존과 동일)
        double curvature = compute_curvature();
        double omega = target_v * curvature;

        geometry_msgs::msg::Accel msg;
        msg.linear.x = target_v;
        msg.angular.z = std::clamp(omega, -M_PI, M_PI);
        accel_pub_->publish(msg);
    }
    // 멤버 변수
    double curr_x_ = 0.0, curr_y_ = 0.0, curr_yaw_ = 0.0, curr_omega_ = 0.0;
    double v_, ld_;
    double other_x_ = 0.0, other_y_ = 0.0, other_yaw_ = 0.0, other_omega_ = 0.0;
    double other_v_ = 0.0;
    std::string other_cav_id_ = "1";
    std::vector<double> path_x_, path_y_;
    std::vector<double> other_path_x_, other_path_y_;
    size_t domain_id_ = 0;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr other_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Accel>::SharedPtr other_accel_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Accel>::SharedPtr accel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    int target_idx_ = 0;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuitNode>());
    rclcpp::shutdown();
    return 0;
}
