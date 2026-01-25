#include <memory>
#include <string>
#include <fstream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <map>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/accel.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "nlohmann/json.hpp"

using json = nlohmann::json;
using namespace std::chrono_literals;

// --- 1. Velocity EKF 클래스 (지연 없는 속도 추정) ---
class VelocityEKF {
public:
    VelocityEKF() {
        is_initialized = false;
        Q.setIdentity(); Q *= 0.01;
        R.setIdentity(); R *= 0.05;
        P.setIdentity();
    }

    void init(double x, double y, double v, double theta) {
        state << x, y, v, theta;
        is_initialized = true;
    }

    void predict(double dt) {
        double v = state(2);
        double theta = state(3);
        state(0) += v * std::cos(theta) * dt;
        state(1) += v * std::sin(theta) * dt;

        Eigen::Matrix4d F = Eigen::Matrix4d::Identity();
        F(0, 2) = std::cos(theta) * dt;
        F(0, 3) = -v * std::sin(theta) * dt;
        F(1, 2) = std::sin(theta) * dt;
        F(1, 3) = v * std::cos(theta) * dt;
        P = F * P * F.transpose() + Q;
    }

    void update(double z_x, double z_y) {
        Eigen::Vector2d z(z_x, z_y);
        Eigen::Matrix<double, 2, 4> H;
        H.setZero(); H(0, 0) = 1.0; H(1, 1) = 1.0;

        Eigen::Vector2d y = z - H * state;
        Eigen::Matrix2d S = H * P * H.transpose() + R;
        Eigen::Matrix<double, 4, 2> K = P * H.transpose() * S.inverse();

        state = state + K * y;
        P = (Eigen::Matrix4d::Identity() - K * H) * P;
    }

    bool is_initialized;
    Eigen::Vector4d state; // [x, y, v, theta]
    Eigen::Matrix4d P, Q;
    Eigen::Matrix2d R;
};

// --- 2. 메인 컨트롤러 노드 ---
class PurePursuitNode : public rclcpp::Node {
public:
    struct Path { std::vector<double> X, Y; };
    struct Point { double x; double y; };
    struct ConflictRule { int other_id; const Path* other_path_ptr; };
    struct SegmentRule { const Path* my_segment_ptr; std::vector<ConflictRule> conflicts; };

    PurePursuitNode() : Node("pure_pursuit_node"), brake_active_(false) {
        const char* env_domain = std::getenv("ROS_DOMAIN_ID");
        domain_id_ = (env_domain != nullptr) ? std::atoi(env_domain) : 1;

        // 하드웨어 파라미터 셋업
        this->declare_parameter("wheelbase", 0.211);
        this->declare_parameter("max_v", 2.0);
        this->declare_parameter("max_kappa", 3.0);
        L_ = this->get_parameter("wheelbase").as_double();
        max_v_ = this->get_parameter("max_v").as_double();
        max_kappa_ = this->get_parameter("max_kappa").as_double();

        if (!init_config_and_paths()) return;
        setup_conflict_rules();

        auto qos = rclcpp::SensorDataQoS();
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/Ego_pose", qos, std::bind(&PurePursuitNode::pose_callback, this, std::placeholders::_1));

        accel_pub_ = this->create_publisher<geometry_msgs::msg::Accel>("/Accel", 10);
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        init_other_subs(qos);

        // 20Hz (50ms) 제어 루프
        timer_ = this->create_wall_timer(50ms, std::bind(&PurePursuitNode::control_loop, this));
    }

private:
    // PID 제동 게인
    double kp_brake_ = 2.5;
    double last_v_error_ = 0.0;

    VelocityEKF ekf_;
    double current_v_est_ = 0.0;
    rclcpp::Time last_pose_time_;

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        rclcpp::Time current_time = msg->header.stamp;
        if (!ekf_.is_initialized) {
            ekf_.init(msg->pose.position.x, msg->pose.position.y, 0.0, msg->pose.orientation.z);
            last_pose_time_ = current_time;
            return;
        }

        double dt = (current_time - last_pose_time_).seconds();
        if (dt > 0.0) {
            ekf_.predict(dt);
            ekf_.update(msg->pose.position.x, msg->pose.position.y);
            current_v_est_ = ekf_.state(2);
        }

        curr_x_ = msg->pose.position.x; curr_y_ = msg->pose.position.y; curr_yaw_ = msg->pose.orientation.z;
        last_pose_time_ = current_time;
        pose_received_ = true;
    }

    void control_loop() {
        if (!pose_received_) return;
        my_idx_ = find_closest_index(my_path_, curr_x_, curr_y_);

        double target_v = check_all_collisions(v_);

        // 1. 브레이크 상태 판별
        bool should_brake = (target_v == 0.0);
        if (should_brake && !brake_active_) brake_active_ = true;
        else if (!should_brake && brake_active_) brake_active_ = false;

        double publish_v;
        double raw_curvature = compute_curvature();

        geometry_msgs::msg::Accel accel_msg;
        geometry_msgs::msg::Twist twist_msg;

        if (brake_active_) {
            // 능동 제동 로직 적용 (EKF 속도 기반 역토크)
            double v_error = 0.0 - current_v_est_;
            double brake_force = kp_brake_ * v_error;

            publish_v = (current_v_est_ > 0.04) ? std::clamp(brake_force, -0.5, 0.0) : 0.0;

            accel_msg.linear.x = 0.0; accel_msg.angular.z = 0.0;
            twist_msg.linear.x = publish_v; twist_msg.angular.z = 0.0;
        }
        else {
            publish_v = target_v;

            // [시뮬레이터용 Accel] 원래 계산 방식
            accel_msg.linear.x = publish_v;
            accel_msg.angular.z = std::clamp(publish_v * raw_curvature, -M_PI, M_PI);

            // [실제 주행용 cmd_vel] 곡률 제약 적용 (v=2.0 -> k=3.0)
            double k_limit = 1.5 * publish_v;
            double clamped_k = std::clamp(raw_curvature, -k_limit, k_limit);

            twist_msg.linear.x = publish_v;
            twist_msg.angular.z = std::clamp(publish_v * clamped_k, -M_PI, M_PI);
        }

        accel_pub_->publish(accel_msg);
        cmd_vel_pub_->publish(twist_msg);
    }

    // --- 충돌 및 경로 데이터 처리 (기존 로직 유지) ---
    double check_all_collisions(double base_v) {
        double out_v = base_v;
        int logical_id = domain_ids[domain_id_];

        for (const auto& segment : my_active_rules_) {
            if (is_on_path(*(segment.my_segment_ptr), curr_x_, curr_y_)) {
                for (const auto& conflict : segment.conflicts) {
                    if (check_hardcoded_collision(conflict.other_id, *(conflict.other_path_ptr))) {
                        out_v = (conflict.other_id < logical_id) ? 0.0 : 0.5;
                        if (logical_id == 2 && is_on_path(path60_52_exit, curr_x_, curr_y_)) out_v = 2.0;
                        break;
                    }
                }
            }
            if (out_v == 0.0) return 0.0;
        }
        if (check_hv_collision_path_based()) return 0.0;
        int c_idx = find_closest_index(circle_path_, curr_x_, curr_y_);
        if (std::hypot(circle_path_.X[c_idx] - curr_x_, circle_path_.Y[c_idx] - curr_y_) < 0.5) out_v = 2.0;
        return std::min(out_v, max_v_);
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

    bool init_config_and_paths() {
        std::string pkg_path = ament_index_cpp::get_package_share_directory("pure_pursuit_controller");
        try {
            YAML::Node config = YAML::LoadFile(pkg_path + "/config/cav_params.yaml");
            for (int i = 1; i <= 4; i++) {
                std::string k = "cav" + std::to_string(i);
                int d_id = config["problem_three"][k]["ros_domain_id"].as<int>(1);
                domain_ids[d_id] = i;
            }
            int my_logical_id = domain_ids[domain_id_];
            std::string k = "cav" + std::to_string(my_logical_id);
            v_ = config["problem_three"][k]["init_speed"].as<double>(1.0);
            ld_ = config["problem_three"][k]["lookahead"].as<double>(0.5);
            my_path_ = load_path_json(pkg_path + "/config/" + config["problem_three"][k]["path"].as<std::string>());

            circle_path_ = load_path_json(pkg_path + "/config/circle.json");
            path21_51 = load_path_json(pkg_path + "/config/21_51.json");
            path22_25 = load_path_json(pkg_path + "/config/22_25_fixed.json");
            path48_58 = load_path_json(pkg_path + "/config/48_58_fixed.json");
            path49_55 = load_path_json(pkg_path + "/config/49_55_fixed.json");
            path51_46 = load_path_json(pkg_path + "/config/21_51_46.json");
            path52_24 = load_path_json(pkg_path + "/config/52_24.json");
            path55_12 = load_path_json(pkg_path + "/config/55_12.json");
            path56_59 = load_path_json(pkg_path + "/config/56_59.json");
            path60_52 = load_path_json(pkg_path + "/config/60_52.json");
            path61_47 = load_path_json(pkg_path + "/config/61_47_fixed.json");
            path60_52_exit = load_path_json(pkg_path + "/config/60_52_fixed.json");
            path8_11 = load_path_json(pkg_path + "/config/8_11_fixed.json");
            path9_56 = load_path_json(pkg_path + "/config/9_56.json");
            return true;
        } catch (...) { return false; }
    }

    void setup_conflict_rules() {
        my_active_rules_.clear();
        int my_logical_id = domain_ids[domain_id_];
        if (my_logical_id == 1) {
            my_active_rules_.push_back({&path21_51, {{3, &path22_25}}});
            my_active_rules_.push_back({&path51_46, {{2, &path60_52}, {2, &path49_55}, {3, &path48_58}}});
            my_active_rules_.push_back({&path9_56,  {{4, &path8_11}}});
            my_active_rules_.push_back({&path56_59, {{2, &path49_55}, {2, &path60_52}, {4, &path61_47}}});
        }
        else if (my_logical_id == 2) {
            my_active_rules_.push_back({&path60_52, {{1, &path56_59}, {1, &path51_46}, {3, &path48_58}}});
            my_active_rules_.push_back({&path52_24, {{3, &path22_25}}});
            my_active_rules_.push_back({&path49_55, {{1, &path51_46}, {1, &path56_59}, {4, &path61_47}}});
            my_active_rules_.push_back({&path55_12, {{4, &path8_11}}});
        }
        else if (my_logical_id == 3) {
            my_active_rules_.push_back({&path22_25, {{1, &path21_51}, {2, &path52_24}}});
            my_active_rules_.push_back({&path48_58, {{1, &path51_46}, {2, &path60_52}}});
        }
        else if (my_logical_id == 4) {
            my_active_rules_.push_back({&path61_47, {{1, &path56_59}, {2, &path49_55}}});
            my_active_rules_.push_back({&path8_11,  {{1, &path9_56}, {2, &path55_12}}});
        }
    }

    void init_other_subs(rclcpp::SensorDataQoS qos) {
        std::string pkg_path = ament_index_cpp::get_package_share_directory("pure_pursuit_controller");
        YAML::Node config = YAML::LoadFile(pkg_path + "/config/cav_params.yaml");
        for (int i = 1; i <= 4; ++i) {
            if (i == domain_ids[domain_id_]) continue;
            std::string cav = "cav" + std::to_string(i);
            int target_domain = config["problem_three"][cav]["ros_domain_id"].as<int>();
            other_subs_[i] = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/CAV" + std::to_string(target_domain) + "_pose", qos, [this, i](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                    this->others_pos_[i] = {msg->pose.position.x, msg->pose.position.y};
                    this->others_received_[i] = true;
                });
        }
        // HV 구독
        other_subs_[19] = this->create_subscription<geometry_msgs::msg::PoseStamped>("/HV_19", qos, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { this->others_pos_[19] = {msg->pose.position.x, msg->pose.position.y}; this->others_received_[19] = true; });
        other_subs_[20] = this->create_subscription<geometry_msgs::msg::PoseStamped>("/HV_20", qos, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { this->others_pos_[20] = {msg->pose.position.x, msg->pose.position.y}; this->others_received_[20] = true; });
    }

    Path load_path_json(const std::string& path) {
        Path p; std::ifstream f(path);
        if (f.is_open()) { json j; f >> j; p.X = j["X"].get<std::vector<double>>(); p.Y = j["Y"].get<std::vector<double>>(); }
        return p;
    }

    int find_closest_index(const Path& path, double x, double y) {
        int idx = 0; double min_d = 1e9;
        for (size_t i = 0; i < path.X.size(); ++i) {
            double d = std::hypot(path.X[i] - x, path.Y[i] - y);
            if (d < min_d) { min_d = d; idx = i; }
        }
        return idx;
    }

    bool is_on_path(const Path& path, double x, double y) {
        double d = std::hypot(path.X[find_closest_index(path, x, y)] - x, path.Y[find_closest_index(path, x, y)] - y);
        return d < 0.2;
    }

    bool check_hardcoded_collision(int other_id, const Path& other_path) {
        if (!others_received_[other_id]) return false;
        int other_idx = find_closest_index(other_path, others_pos_[other_id].x, others_pos_[other_id].y);
        double my_accum = 0.0;
        for (int i = my_idx_; i < (int)my_path_.X.size() - 1 && my_accum < 1.0; ++i) {
            my_accum += std::hypot(my_path_.X[i+1]-my_path_.X[i], my_path_.Y[i+1]-my_path_.Y[i]);
            double other_accum = 0.0;
            for (int j = other_idx; j < (int)other_path.X.size() - 1 && other_accum < 1.5; ++j) {
                other_accum += std::hypot(other_path.X[j+1]-other_path.X[j], other_path.Y[j+1]-other_path.Y[j]);
                if (std::hypot(my_path_.X[i] - other_path.X[j], my_path_.Y[i] - other_path.Y[j]) < 0.25) {
                    if (my_accum > other_accum + 0.15) return true;
                    if (std::abs(my_accum - other_accum) <= 0.15 && domain_ids[domain_id_] > other_id) return true;
                }
            }
        }
        return false;
    }

    bool check_hv_collision_path_based() {
        for (int id : {19, 20}) {
            if (!others_received_[id]) continue;
            // HV 경로 탐색 로직 (필요시 추가 구현 가능)
        }
        return false;
    }

    // 멤버 변수
    size_t domain_id_; int my_idx_ = 0;
    double curr_x_, curr_y_, curr_yaw_, v_, ld_, L_, max_v_, max_kappa_;
    bool pose_received_ = false; bool brake_active_;
    Path my_path_, circle_path_, path8_11, path9_56, path21_51, path22_25, path48_58, path49_55, path51_46, path52_24, path55_12, path56_59, path60_52, path61_47, path60_52_exit;
    std::map<int, Point> others_pos_; std::map<int, bool> others_received_;
    std::map<int, Path> others_paths_;
    std::vector<SegmentRule> my_active_rules_;
    std::map<int, int> domain_ids;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    std::map<int, rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> other_subs_;
    rclcpp::Publisher<geometry_msgs::msg::Accel>::SharedPtr accel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuitNode>());
    rclcpp::shutdown();
    return 0;
}
