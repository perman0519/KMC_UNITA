#include <memory>
#include <string>
#include <fstream>
#include <vector>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "std_msgs/msg/float64.hpp"  // <--- 추가됨
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "nlohmann/json.hpp"

// 직접 쿼터니언을 사용하므로 tf2 헤더는 삭제해도 무방합니다.
#include "geometry_msgs/msg/quaternion.hpp"

using json = nlohmann::json;

class PurePursuitVizNode : public rclcpp::Node {
public:
    PurePursuitVizNode() : Node("pure_pursuit_viz_node") {
        this->declare_parameter("lookahead_distance", 0.7);
        this->declare_parameter("path_file", "CAV_01_1.json");

        ld_ = this->get_parameter("lookahead_distance").as_double();
        std::string path_file = this->get_parameter("path_file").as_string();

        if (!load_path(path_file)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load path file!");
        }

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/viz/global_path", 10);
        target_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/viz/target_point", 10);
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/viz/ego_pose_converted", 10);
        error_pub_ = this->create_publisher<std_msgs::msg::Float64>("/viz/lateral_error", 10);

        auto qos_profile = rclcpp::SensorDataQoS();
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/Ego_pose", qos_profile,
            std::bind(&PurePursuitVizNode::pose_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(std::chrono::seconds(1), [this]() { publish_path(); });

        RCLCPP_INFO(this->get_logger(), "Visualization Node Started. Ld: %.2f", ld_);
    }

private:
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        double curr_x = msg->pose.position.x;
        double curr_y = msg->pose.position.y;

        // 1. 가장 가까운 경로 점(Closest Point) 찾기
        double min_dist = std::numeric_limits<double>::max();
        int closest_idx = 0;

        for (size_t i = 0; i < path_x_.size(); ++i) {
            double d = std::hypot(path_x_[i] - curr_x, path_y_[i] - curr_y); // sqrt(pow+pow)보다 성능 좋음
            if (d < min_dist) {
                min_dist = d;
                closest_idx = i;
            }
        }

        // 2. Lateral Error(CTE) 방향 판별 및 발행
        double error_val = min_dist;
        if (closest_idx < (int)path_x_.size() - 1) {
            double dx = path_x_[closest_idx + 1] - path_x_[closest_idx];
            double dy = path_y_[closest_idx + 1] - path_y_[closest_idx];
            double ux = curr_x - path_x_[closest_idx];
            double uy = curr_y - path_y_[closest_idx];
            // Cross product를 통해 좌/우 판별
            if ((dx * uy - dy * ux) < 0) error_val *= -1.0;
        }

        std_msgs::msg::Float64 error_msg;
        error_msg.data = error_val;
        error_pub_->publish(error_msg);

        // 3. Pose 시각화 (Euler -> Quaternion)
        geometry_msgs::msg::PoseStamped converted_pose;
        converted_pose.header = msg->header;
        converted_pose.header.frame_id = "map";
        converted_pose.pose.position = msg->pose.position;
        converted_pose.pose.orientation = euler_to_quaternion(
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z
        );
        pose_pub_->publish(converted_pose);

        // 4. Look-ahead Target 시각화
        find_and_publish_target(curr_x, curr_y, closest_idx);
    }

    // closest_idx를 인자로 받아 연산 중복 감소
    void find_and_publish_target(double x, double y, int closest_idx) {
        int target_idx = -1;
        for (size_t i = closest_idx; i < path_x_.size(); ++i) {
            double d = std::hypot(path_x_[i] - x, path_y_[i] - y);
            if (d >= ld_) {
                target_idx = i;
                break;
            }
        }
        if (target_idx == -1) target_idx = path_x_.size() - 1;

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.pose.position.x = path_x_[target_idx];
        marker.pose.position.y = path_y_[target_idx];
        marker.pose.position.z = 0.2;
        marker.scale.x = 0.3; marker.scale.y = 0.3; marker.scale.z = 0.3;
        marker.color.a = 0.8; marker.color.r = 1.0; marker.color.g = 1.0; marker.color.b = 0.0;
        target_pub_->publish(marker);
    }

    // 직접 구현한 Quaternion 변환 로직
    geometry_msgs::msg::Quaternion euler_to_quaternion(double roll, double pitch, double yaw) {
        double cy = std::cos(yaw * 0.5);
        double sy = std::sin(yaw * 0.5);
        double cp = std::cos(pitch * 0.5);
        double sp = std::sin(pitch * 0.5);
        double cr = std::cos(roll * 0.5);
        double sr = std::sin(roll * 0.5);

        geometry_msgs::msg::Quaternion q;
        q.w = cr * cp * cy + sr * sp * sy;
        q.x = sr * cp * cy - cr * sp * sy;
        q.y = cr * sp * cy + sr * cp * sy;
        q.z = cr * cp * sy - sr * sp * cy;
        return q;
    }

    bool load_path(const std::string& filename) {
        std::string pkg_path = ament_index_cpp::get_package_share_directory("pure_pursuit_controller");
        std::string full_path = pkg_path + "/config/" + filename;
        std::ifstream f(full_path);
        if (!f.is_open()) return false;
        json j;
        f >> j;
        path_x_ = j["X"].get<std::vector<double>>();
        path_y_ = j["Y"].get<std::vector<double>>();
        return true;
    }

    void publish_path() {
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = "map";
        for (size_t i = 0; i < path_x_.size(); ++i) {
            geometry_msgs::msg::PoseStamped p;
            p.pose.position.x = path_x_[i];
            p.pose.position.y = path_y_[i];
            path_msg.poses.push_back(p);
        }
        path_pub_->publish(path_msg);
    }

    double ld_;
    std::vector<double> path_x_, path_y_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr target_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr error_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuitVizNode>());
    rclcpp::shutdown();
    return 0;
}
