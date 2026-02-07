#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <fstream>
#include <string>
#include <vector>
#include <iostream>

#include "map_visualizer/json.hpp"

using json = nlohmann::json;

class MapVisualizerNode : public rclcpp::Node
{
public:
  MapVisualizerNode()
  : Node("map_visualizer_node")
  {
    publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("map_markers", 10);

    // Timer to publish map periodically (e.g., every 2 seconds, as it is static)
    timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&MapVisualizerNode::publish_map, this));

    load_map();
  }

private:
  void load_map()
  {
    std::string package_share_directory;
    try {
      package_share_directory = ament_index_cpp::get_package_share_directory("simulator");
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get simulator package share directory: %s", e.what());
      return;
    }

    std::string map_path = package_share_directory + "/resource/assets/path.json";
    RCLCPP_INFO(this->get_logger(), "Loading map from: %s", map_path.c_str());

    std::ifstream file(map_path);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open map file");
      return;
    }

    json data;
    try {
      file >> data;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to parse JSON: %s", e.what());
      return;
    }

    parse_path(data);
  }

  void create_lane_marker(const json &pathObj, int &id_counter)
  {
    if (!pathObj.contains("X") || !pathObj.contains("Y")) {
        return;
    }

    std::vector<double> xs = pathObj["X"].get<std::vector<double>>();
    std::vector<double> ys = pathObj["Y"].get<std::vector<double>>();

    if (xs.size() != ys.size()) {
        RCLCPP_WARN(this->get_logger(), "X and Y size mismatch in lane data");
        return;
    }

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "lanes";
    marker.id = id_counter++;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.scale.x = 0.2; // Line width

    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 1.0;

    marker.pose.orientation.w = 1.0;

    for (size_t i = 0; i < xs.size(); i++) {
      geometry_msgs::msg::Point p;
      p.x = xs[i];
      p.y = ys[i];
      p.z = 0.0;
      marker.points.push_back(p);
    }

    marker_array_.markers.push_back(marker);
  }

  void parse_path(const json &data)
  {
    int id_counter = 0;
    for (const auto &item : data) {
      if (item.is_array()) {
        for (const auto &pathObj : item) {
          create_lane_marker(pathObj, id_counter);
        }
      } else if (item.is_object() && item.contains("X") && item.contains("Y")) {
        create_lane_marker(item, id_counter);
      }
    }
    RCLCPP_INFO(this->get_logger(), "Loaded %zu lanes", marker_array_.markers.size());
  }

  void publish_map()
  {
    if (!marker_array_.markers.empty()) {
        // Update timestamp
        for (auto &marker : marker_array_.markers) {
            marker.header.stamp = this->now();
        }
        publisher_->publish(marker_array_);
    }
  }

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  visualization_msgs::msg::MarkerArray marker_array_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapVisualizerNode>());
  rclcpp::shutdown();
  return 0;
}
