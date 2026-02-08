#include <memory>
#include <string>
#include <fstream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <yaml-cpp/yaml.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp" // 실제 차량용 (cmd_vel)
#include "geometry_msgs/msg/accel.hpp" // 시뮬레이터용 (Accel)
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "nlohmann/json.hpp"

using json = nlohmann::json;
using namespace std::chrono_literals;

class PurePursuitNode : public rclcpp::Node {
public:
    PurePursuitNode() : Node("pure_pursuit_node") {
        // 1. 파라미터 선언 및 하드웨어 설정 (KMC 사양)
        this->declare_parameter("wheelbase", 0.211); // 축거: 0.211m
        this->declare_parameter("max_v", 2.0);       // 최대 속도: 2.0m/s
        this->declare_parameter("lookahead_default", 0.5);

        L_ = this->get_parameter("wheelbase").as_double();
        max_v_ = this->get_parameter("max_v").as_double();

        // 2. 경로 및 설정 로드
        if (!init_path()) {
            RCLCPP_ERROR(this->get_logger(), "경로 파일을 로드하지 못했습니다. 설정을 확인하세요.");
            return;
        }

        auto qos_profile = rclcpp::SensorDataQoS();

        // 3. 구독자(Subscribers) 설정
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/Ego_pose", qos_profile, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                this->curr_x_ = msg->pose.position.x;
                this->curr_y_ = msg->pose.position.y;
                this->curr_yaw_ = msg->pose.orientation.z; // 시뮬레이터 특이사항: z가 yaw
            });

        other_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/CAV" + other_cav_id_ + "_pose", qos_profile, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                this->other_x_ = msg->pose.position.x;
                this->other_y_ = msg->pose.position.y;
            });

        // 4. 발행자(Publishers) 설정 - Dual Output
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        accel_pub_ = this->create_publisher<geometry_msgs::msg::Accel>("/Accel", 10);

        // 5. 제어 루프 타이머 (10Hz)
        timer_ = this->create_wall_timer(100ms, std::bind(&PurePursuitNode::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "KMC Dual Controller Node Started. (L: %.3f, Max V: %.1f)", L_, max_v_);
    }

private:
    struct CollisionInfo {
        bool collision_predicted = false;
        double dist_to_collision_ego = 1e9;
        double dist_to_collision_other = 1e9;
    };



    inline double computeTargetSpeed(double curvature)
    {
        double kappa = std::abs(curvature);

        double v_max = 1.7; // 직선
        double v_min = 0.8;    // 급커브
        double kappa_max = 3.0; //


        double ratio = kappa / kappa_max;
        // if (ratio > 1.0)
        //     ratio = 1.0;

        // quadratic 감속
        ratio = ratio * ratio;


        RCLCPP_INFO(this->get_logger(),"kappa : %.3f,ratio : %.3f",kappa,ratio);


        return v_max - (v_max - v_min) * ratio;
    }



    // --- 핵심 제어 로직 ---
    void control_loop() {
        // 충돌 위험 판단
        CollisionInfo col = check_collision_detail();
        // double target_v = std::min(v_, max_v_);

        double target_v=0.0;

        if (col.collision_predicted) {
            // 양보 로직: 내가 더 멀리 있거나 ID가 높을 때(거리가 비슷하면) 정지
            if (col.dist_to_collision_ego > col.dist_to_collision_other) {
                target_v = 0.0;
            } else if (std::abs(col.dist_to_collision_ego - col.dist_to_collision_other) < 0.15) {
                if (domain_id_ != 1) target_v = 0.0;
            }
        }

        // Pure Pursuit 조향 계산
        double curvature = compute_curvature();

        // 곡률 기반 속도
        target_v = computeTargetSpeed(curvature);
        target_v = std::min(target_v, max_v_);

        target_v = std::clamp(target_v,0.8,1.7);

        double omega = target_v * curvature;
        double max = 3.0;
        double mav_yaw = target_v * max;
        double clamped_omega = std::clamp(omega, -mav_yaw, mav_yaw);

        // 메시지 생성 및 동시 발행
        geometry_msgs::msg::Twist twist_msg;
        twist_msg.linear.x = target_v;
        twist_msg.angular.z = clamped_omega;

        geometry_msgs::msg::Accel accel_msg;
        accel_msg.linear.x = target_v;
        accel_msg.angular.z = clamped_omega;

        cmd_vel_pub_->publish(twist_msg);
        accel_pub_->publish(accel_msg);
    }

    double compute_curvature() {
        int my_idx = find_closest_index(path_x_, path_y_, curr_x_, curr_y_);
        int target_idx = -1;

        // Look-ahead distance(ld_)만큼 앞선 목표점 찾기
        for (size_t i = my_idx; i < path_x_.size(); ++i) {
            if (std::hypot(path_x_[i] - curr_x_, path_y_[i] - curr_y_) >= ld_) {
                target_idx = i;
                break;
            }
        }
        if (target_idx == -1) target_idx = path_x_.size() - 1;

        double dx = path_x_[target_idx] - curr_x_;
        double dy = path_y_[target_idx] - curr_y_;

        // 로컬 좌표계 변환 및 곡률 계산: kappa = 2*y / Ld^2
        double local_y = -std::sin(curr_yaw_) * dx + std::cos(curr_yaw_) * dy;
        return (2.0 * local_y) / (ld_ * ld_);
    }

    CollisionInfo check_collision_detail() {
        CollisionInfo info;
        if (path_x_.empty() || other_path_x_.empty()) return info;

        int my_idx = find_closest_index(path_x_, path_y_, curr_x_, curr_y_);
        int other_idx = find_closest_index(other_path_x_, other_path_y_, other_x_, other_y_);

        double my_accum = 0.0;
        for (int i = my_idx; i < (int)path_x_.size() - 1 && my_accum < 1.5; ++i) {
            my_accum += std::hypot(path_x_[i+1] - path_x_[i], path_y_[i+1] - path_y_[i]);
            double other_accum = 0.0;
            for (int j = other_idx; j < (int)other_path_x_.size() - 1 && other_accum < 1.5; ++j) {
                other_accum += std::hypot(other_path_x_[j+1] - other_path_x_[j], other_path_y_[j+1] - other_path_y_[j]);
                if (std::hypot(path_x_[i] - other_path_x_[j], path_y_[i] - other_path_y_[j]) < 0.25) {
                    info.collision_predicted = true;
                    info.dist_to_collision_ego = my_accum;
                    info.dist_to_collision_other = other_accum;
                    return info;
                }
            }
        }
        return info;
    }

    // --- 유틸리티 함수 ---
    int find_closest_index(const std::vector<double>& px, const std::vector<double>& py, double x, double y) {
        int idx = 0; double min_d = 1e9;
        for (size_t i = 0; i < px.size(); ++i) {
            double d = std::hypot(px[i] - x, py[i] - y);
            if (d < min_d) { min_d = d; idx = i; }
        }
        return idx;
    }

    bool init_path() {
        std::string pkg_path = ament_index_cpp::get_package_share_directory("pure_pursuit_controller");
        try {
            YAML::Node config = YAML::LoadFile(pkg_path + "/config/cav_params.yaml");
            const char* env_domain = std::getenv("ROS_DOMAIN_ID");
            domain_id_ = (env_domain != nullptr) ? std::atoi(env_domain) : 1;

            std::string key = (domain_id_ == 1) ? "cav1" : "cav2";
            other_cav_id_ = (domain_id_ == 1) ? "2" : "1";

            auto p = config["problem_one"][key];
            v_ = p["init_speed"].as<double>(1.0);
            ld_ = p["lookahead"].as<double>(0.5);

            std::string my_json = pkg_path + "/config/" + p["path"].as<std::string>();
            std::string other_json = pkg_path + "/config/" + config["problem_one"][(domain_id_ == 1 ? "cav2" : "cav1")]["path"].as<std::string>();

            path_x_ = load_json_vec(my_json, "X");
            path_y_ = load_json_vec(my_json, "Y");
            other_path_x_ = load_json_vec(other_json, "X");
            other_path_y_ = load_json_vec(other_json, "Y");
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Config Load Error: %s", e.what());
            return false;
        }
    }

    std::vector<double> load_json_vec(const std::string& path, const std::string& key) {
        std::ifstream f(path);
        json j; f >> j;
        return j[key].get<std::vector<double>>();
    }

    // 멤버 변수
    double L_, max_v_, v_, ld_;
    double curr_x_, curr_y_, curr_yaw_;
    double other_x_, other_y_;
    int domain_id_;
    std::string other_cav_id_;
    std::vector<double> path_x_, path_y_, other_path_x_, other_path_y_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_, other_pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Accel>::SharedPtr accel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuitNode>());
    rclcpp::shutdown();
    return 0;
}
