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
        } else {
            RCLCPP_INFO(this->get_logger(), "초기화 성공");
            RCLCPP_INFO(this->get_logger(), "v=%.2f, ld=%.2f", v_, ld_);
            RCLCPP_INFO(this->get_logger(), "모든 경로 로드 완료");
            RCLCPP_INFO(this->get_logger(), "1차선 경로 점 수: %zu", lanes_path_[1].X.size());
            RCLCPP_INFO(this->get_logger(), "2차선 경로 점 수: %zu", lanes_path_[2].X.size());
            RCLCPP_INFO(this->get_logger(), "3차선 경로 점 수: %zu", lanes_path_[3].X.size());
        }
        auto qos = rclcpp::SensorDataQoS();

        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/Ego_pose", qos, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                this->curr_x_ = msg->pose.position.x;
                this->curr_y_ = msg->pose.position.y;
                this->curr_yaw_ = msg->pose.orientation.z; // yaw(rad)

                pose_received_ = true;
                pose_count_++;
            });

        for (int i = 19; i <= 22; ++i) {
            std::string topic = "/HV_" + std::to_string(i);
            line3_pose_sub_[i] = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                topic, qos, [this, i](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                    this->line3_hvs_x_[i] = msg->pose.position.x;
                    this->line3_hvs_y_[i] = msg->pose.position.y;
                });
        }
        for (int i = 23; i <= 30; ++i) {
            std::string topic = "/HV_" + std::to_string(i);
            line2_pose_sub_[i] = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                topic, qos, [this, i](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                    this->line2_hvs_x_[i] = msg->pose.position.x;
                    this->line2_hvs_y_[i] = msg->pose.position.y;
                });
        }
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
        lane_28_31_1_3_5_path_front  = load_path_json(pkg_path + "/config/28_31_1_3_5_front.json");
        lane_28_31_1_3_5_path_cross  = load_path_json(pkg_path + "/config/28_31_1_3_5_cross.json");
        lane_28_31_1_3_5_path_back   = load_path_json(pkg_path + "/config/28_31_1_3_5_back.json");
        lane_28_31_1_3_5_path_change = load_path_json(pkg_path + "/config/28_31_1_3_5_change.json");
        lane_28_31_1_3_5_path_lane2  = load_path_json(pkg_path + "/config/28_31_1_3_5_lane2.json");

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

    // 한 단계 차선 변경만 허용 (1<->2, 2<->3)
    bool try_set_lane_stepwise(int desired_lane) {
        if (desired_lane < 1 || desired_lane > 3) return false;
        if (std::abs(desired_lane - current_lane_) != 1) return false; // 1->3, 3->1 금지
        current_lane_ = desired_lane;
        return true;
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

        int search_steps = 105;
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
        if (!pose_received_) return;

        if (pose_count_ < pose_start_threshold_) {
            publish_cmd(0.0, 0.0);
            return;
        }

        double dist_to_lane = min_dist_to_path(lanes_path_[current_lane_]);
        if (dist_to_lane > path_lock_dist_) {
            publish_cmd(0.0, 0.0);
            return;
        }

        double target_v = v_;

        bool on_forbidden_lane3_path    = is_on_path(lane_27_30_1_3_6_path_, 0.1);
        bool on_force_lane2_path_back   = is_on_path(lane_28_31_1_3_5_path_back, 0.1);
        bool on_force_lane2_path_change = is_on_path(lane_28_31_1_3_5_path_change, 0.1);

        // 28~31 zone (넓게)
        bool in_28_31_zone = is_on_path(lane_28_31_1_3_5_path_front, 0.5);

        // 강제 2차선 복귀(뒷부분) - 한 단계만
        if (on_force_lane2_path_back && current_lane_ != 2) {
            if (try_set_lane_stepwise(2)) {
                RCLCPP_INFO(this->get_logger(), "강제 복귀: 2차선으로 한 단계 이동");
            }
        }

        // ===== 장애물 처리 =====
        if (is_lane_blocked(current_lane_)) {

            if (current_lane_ == 2) {

                if (in_28_31_zone) {
                    // [MOD3] 28~31 구간에서는 "1차선만" 강제 이동 (2->1)
                    //        -> 어떤 조건(금지 플래그/3차선 여부)과 무관하게 2->1을 최우선으로 강제
                    if (try_set_lane_stepwise(1)) {  // [MOD3]
                        RCLCPP_INFO(this->get_logger(),
                                    "28~31 구간: 2차선 막힘 -> 1차선으로 강제 이동(2->1)"); // [MOD3]
                    }

                    // [MOD3] 이동했는데 1차선도 막혀있으면 정지
                    if (is_lane_blocked(1)) { // [MOD3]
                        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                             "28~31 구간: 1차선도 막힘 -> 정지"); // [MOD3]
                        target_v = 0.0; // [MOD3]
                    }
                }
                else {
                    // 일반 구간: 기존 로직 유지 (2->3 우선, 안되면 2->1)
                    if (!on_forbidden_lane3_path && !is_lane_blocked(3)) {
                        if (try_set_lane_stepwise(3)) {
                            RCLCPP_INFO(this->get_logger(), "2차선 막혀서 3차선으로 변경(2->3)");
                        }
                    }
                    else if (!is_lane_blocked(1)) {
                        if (try_set_lane_stepwise(1)) {
                            RCLCPP_INFO(this->get_logger(), "2차선 막혀서 1차선으로 변경(2->1)");
                        }
                    }
                    else {
                        target_v = 0.0;
                    }
                }
            }
            else {
                // 1/3차선은 오직 2차선으로만 복귀 (한 단계)
                if (!is_lane_blocked(2) && !is_lane_behind_blocked(2)) {
                    if (try_set_lane_stepwise(2)) {
                        RCLCPP_INFO(this->get_logger(), "현재 차선 막힘 -> 2차선 복귀(한 단계)");
                    }
                } else {
                    target_v = 0.0;
                }
            }
        }

        // 합류 지점: 3->2 (한 단계) + 정지
        if (on_force_lane2_path_change && current_lane_ == 3) {
            RCLCPP_INFO(this->get_logger(), "합류 지점: 2차선 변경(3->2) 및 일시 정지");
            try_set_lane_stepwise(2);
            target_v = 0.0;
        }

        if (is_lane_behind_blocked(current_lane_)) {
            target_v = 2.0;
        }

        double curvature = compute_curvature(lanes_path_[current_lane_]);
        double omega = target_v * curvature;
        omega = std::clamp(omega, -omega_max_, omega_max_);

        publish_cmd(target_v, omega);
    }

    void publish_cmd(double v, double omega) {
        geometry_msgs::msg::Accel msg;
        msg.linear.x = v;
        msg.angular.z = omega;
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

    bool pose_received_ = false;
    int pose_count_ = 0;
    int pose_start_threshold_ = 5;

    double omega_max_ = 3.1;
    double path_lock_dist_ = 0.8;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Quiz2>());
    rclcpp::shutdown();
    return 0;
}