#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <string>
#include "map_visualizer/simpleIni.h"

class CavVisualizerNode : public rclcpp::Node {
public:
    CavVisualizerNode() : Node("cav_visualizer_node") {
        // 1. 파라미터 선언 및 즉시 로드
        this->declare_parameter("topic_name", "/CAV_01");
        this->declare_parameter("logical_id", 1);
        this->declare_parameter("display_name", "CAV_1");

        topic_name_ = this->get_parameter("topic_name").as_string();
        logical_id_ = this->get_parameter("logical_id").as_int();
        display_name_ = this->get_parameter("display_name").as_string();

        // 2. 토픽 및 발행자 설정
        std::string marker_topic = "cav" + std::to_string(logical_id_) + "/marker";
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic, 10);

        // 3. 설정 로드 및 오프셋 사전 계산
        load_config();
        setup_marker_templates();

        // 4. 구독 설정 (람다를 사용하여 바인드 오버헤드 방지)
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            topic_name_, rclcpp::SensorDataQoS(),
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { pose_callback(msg); });

        RCLCPP_INFO(this->get_logger(), "Optimized Visualizer [Slot %d] for %s initialized.", logical_id_, topic_name_.c_str());
    }

private:
    void setup_marker_templates() {
        // 마커 배열에 공간 2개 생성
        markers_msg_.markers.clear();
        markers_msg_.markers.resize(2);

        // 1. 차량 본체 마커 설정 (첫 번째 요소)
        auto& car = markers_msg_.markers[0];
        car.header.frame_id = "map";
        car.ns = "cav_body";
        car.id = 0;
        car.type = visualization_msgs::msg::Marker::CUBE;
        car.action = visualization_msgs::msg::Marker::ADD;
        car.scale.x = lengthF_ + lengthR_;
        car.scale.y = lengthW_ * 2.0;
        car.scale.z = 0.15;
        car.color.a = 1.0;

        // 색상 로직 (중괄호 포함 확인)
        if (logical_id_ <= 4) {
            car.color.r = 0.0; car.color.g = 0.5; car.color.b = 1.0; // Blue
        } else {
            car.color.r = 1.0; car.color.g = 0.2; car.color.b = 0.2; // Red
        }

        // 2. 텍스트 마커 설정 (두 번째 요소)
        auto& text = markers_msg_.markers[1];
        text.header.frame_id = "map";
        text.ns = "cav_label";
        text.id = 1;
        text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::msg::Marker::ADD;
        text.scale.z = 0.4;
        text.text = display_name_;
        text.color.r = 1.0; text.color.g = 1.0; text.color.b = 1.0; // White
        text.color.a = 1.0;

        // 오프셋 사전 계산
        double center_offset_x = (lengthF_ - lengthR_) / 2.0;
        precomputed_offset_ = tf2::Vector3(center_offset_x, 0.0, car.scale.z / 2.0);
    }
    
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (!msg) return;

        // 1. 입력받은 RPY(Roll, Pitch, Yaw)를 이용해 tf2 쿼터니언 생성
        tf2::Quaternion q;
        // msg->pose.orientation.x = Roll
        // msg->pose.orientation.y = Pitch
        // msg->pose.orientation.z = Yaw (Heading)
        q.setRPY(
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z
        );

        // 2. 사전 계산된 오프셋을 현재 회전에 맞춰 적용
        tf2::Vector3 pos(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        tf2::Vector3 rotated_offset = tf2::quatRotate(q, precomputed_offset_);
        tf2::Vector3 final_pos = pos + rotated_offset;

        // 3. 마커 메시지 업데이트 (RViz용 geometry_msgs::msg::Quaternion으로 변환)
        auto now = this->now();
        geometry_msgs::msg::Quaternion q_msg = tf2::toMsg(q);

        // Body Marker 업데이트
        auto& car = markers_msg_.markers[0];
        car.header.stamp = now;
        car.pose.position.x = final_pos.x();
        car.pose.position.y = final_pos.y();
        car.pose.position.z = final_pos.z();
        car.pose.orientation = q_msg; // 변환된 쿼터니언 주입

        // Text Marker 업데이트
        auto& text = markers_msg_.markers[1];
        text.header.stamp = now;
        text.pose.position.x = final_pos.x();
        text.pose.position.y = final_pos.y();
        text.pose.position.z = final_pos.z() + 0.5;
        // 텍스트는 orientation을 따로 업데이트하지 않아도 TEXT_VIEW_FACING이라 카메라를 봅니다.

        // 4. 발행
        marker_pub_->publish(markers_msg_);
    }

    void load_config() {
        try {
            std::string pkg_share = ament_index_cpp::get_package_share_directory("simulator");
            CSimpleIniA ini;
            if (ini.LoadFile((pkg_share + "/resource/config.ini").c_str()) >= 0) {
                lengthF_ = ini.GetDoubleValue("vehicle", "lengthF", 0.17);
                lengthR_ = ini.GetDoubleValue("vehicle", "lengthR", 0.16);
                lengthW_ = ini.GetDoubleValue("vehicle", "lengthW", 0.075);
                return;
            }
        } catch (...) {}
        lengthF_ = 0.17; lengthR_ = 0.16; lengthW_ = 0.075;
    }

    // 멤버 변수
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

    visualization_msgs::msg::MarkerArray markers_msg_; // 멤버 변수로 유지하여 재사용
    tf2::Vector3 precomputed_offset_; // 사전 계산된 오프셋 변수

    std::string topic_name_, display_name_;
    int logical_id_;
    double lengthF_, lengthR_, lengthW_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CavVisualizerNode>());
    rclcpp::shutdown();
    return 0;
}
