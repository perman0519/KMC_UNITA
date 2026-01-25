#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <string>
#include <vector>
#include <iostream>

#include "map_visualizer/simpleIni.h"

class CavVisualizerNode : public rclcpp::Node
{
public:
  CavVisualizerNode()
  : Node("cav_visualizer_node")
  {
    this->declare_parameter("topic_name", "/CAV_01");
    this->declare_parameter("vehicle_id", 1); // For unique marker ID

    topic_name_ = this->get_parameter("topic_name").as_string();
    vehicle_id_ = this->get_parameter("vehicle_id").as_int();

    marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("cav_markers", 10);

    // Subscribe to the PoseStamped topic
    // CavVisualizerNode 생성자 내부
  pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    topic_name_,
    rclcpp::SensorDataQoS(), // QoS를 SensorData(Best Effort)로 변경
    std::bind(&CavVisualizerNode::pose_callback, this, std::placeholders::_1));
    load_config();
  }

private:
  void load_config()
  {
    std::string package_share_directory;
    try {
      package_share_directory = ament_index_cpp::get_package_share_directory("simulator");
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get simulator package share directory: %s", e.what());
      // Fallback defaults
      lengthF_ = 0.17;
      lengthR_ = 0.16;
      lengthW_ = 0.075;
      return;
    }

    std::string config_path = package_share_directory + "/resource/config.ini";
    RCLCPP_INFO(this->get_logger(), "Loading vehicle config from: %s", config_path.c_str());

    CSimpleIniA ini;
    ini.SetUnicode();
    SI_Error rc = ini.LoadFile(config_path.c_str());
    if (rc < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load config.ini");
        return;
    }

    // Default values if not found
    lengthF_ = ini.GetDoubleValue("vehicle", "lengthF", 0.17);
    lengthR_ = ini.GetDoubleValue("vehicle", "lengthR", 0.16);
    lengthW_ = ini.GetDoubleValue("vehicle", "lengthW", 0.075);

    RCLCPP_INFO(this->get_logger(), "Vehicle dimensions: LF=%.3f, LR=%.3f, LW=%.3f", lengthF_, lengthR_, lengthW_);
  }

  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map"; // Assuming map frame, or use msg->header.frame_id if appropriate
    marker.header.stamp = this->now();
    marker.ns = "cavs";
    marker.id = vehicle_id_;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Dimensions
    // Length = LF + LR
    // Width = 2 * LW
    // Height = Arbitrary (e.g., 0.15 to look like a small car/box)
    marker.scale.x = lengthF_ + lengthR_;
    marker.scale.y = lengthW_ * 2.0;
    marker.scale.z = 0.15; // Approximate height

    // Color (Blue-ish for CAVs)
    marker.color.r = 0.0;
    marker.color.g = 0.5;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    // Position
    // The received pose is the center of the rear axle? Or center of gravity?
    // In vehicle.cpp: points are relative to (0,0). Front is +LF, Rear is -LR.
    // So the origin is between the axles?
    // If the tracked pose corresponds to this origin (0,0), then we need to offset the Cube
    // because the Cube's origin is its geometric center.
    // Geometric center X relative to pose origin: (LF - LR) / 2
    // Geometric center Y relative to pose origin: 0
    // Geometric center Z relative to pose origin: height / 2 (assuming pose is on ground)

    double center_offset_x = (lengthF_ - lengthR_) / 2.0;

    // Extract Euler angles from the weird orientation field
    // User said: x=Roll, y=Pitch, z=Yaw
    double roll = msg->pose.orientation.x;
    double pitch = msg->pose.orientation.y;
    double yaw = msg->pose.orientation.z;

    // Create Quaternion from RPY
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);

    // Apply rotation to the offset
    // We need to transform the local center offset (center_offset_x, 0, 0) by the orientation
    // to get the global offset.
    // Actually, it's easier to set the marker pose to the vehicle pose,
    // and if Marker supported pivot points, we'd use that.
    // Since Marker is centered, we must compute the center position in global frame.

    // Global Position = Pose Position + Rotation * Local Offset
    // Local Offset = (center_offset_x, 0, marker.scale.z / 2.0)

    tf2::Vector3 pos(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    tf2::Vector3 local_offset(center_offset_x, 0.0, marker.scale.z / 2.0);
    tf2::Vector3 rotated_offset = tf2::quatRotate(q, local_offset);
    tf2::Vector3 final_pos = pos + rotated_offset;

    marker.pose.position.x = final_pos.x();
    marker.pose.position.y = final_pos.y();
    marker.pose.position.z = final_pos.z();

    marker.pose.orientation = tf2::toMsg(q);

    marker_publisher_->publish(marker);

    // Also publish a Text marker for ID
    visualization_msgs::msg::Marker text_marker;
    text_marker.header = marker.header;
    text_marker.ns = "cav_ids";
    text_marker.id = vehicle_id_;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    text_marker.scale.z = 0.3; // Text height
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.color.a = 1.0;
    text_marker.pose.position.x = final_pos.x();
    text_marker.pose.position.y = final_pos.y();
    text_marker.pose.position.z = final_pos.z() + 0.3;
    text_marker.text = "CAV_" + std::to_string(vehicle_id_);

    marker_publisher_->publish(text_marker);
  }

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;

  std::string topic_name_;
  int vehicle_id_;

  double lengthF_;
  double lengthR_;
  double lengthW_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CavVisualizerNode>());
  rclcpp::shutdown();
  return 0;
}
