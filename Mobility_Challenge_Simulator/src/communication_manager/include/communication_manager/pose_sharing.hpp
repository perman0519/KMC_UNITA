#ifndef COMMUNICATION_MANAGER__POSE_SHARING_HPP_
#define COMMUNICATION_MANAGER__POSE_SHARING_HPP_

#pragma once
#include "communication_base.hpp"

#include <fstream>
#include <vector>
#include <cmath>
#include <ament_index_cpp/get_package_share_directory.hpp>

class PoseSharing : public CommunicationBase {
public:
  PoseSharing(const rclcpp::Node::SharedPtr& node){
      node_ = node;

      timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(25),
            std::bind(&PoseSharing::spin_executor, this)
        );
  }

  void activate() override {
    if (active_) return;
    active_ = true;
    RCLCPP_INFO(node_->get_logger(), "[Pose Sharing] activate");
    vec_topics_init();
    pose_communication();

    spinning_ = true;
  }

  void deactivate() override {
      if (!active_) return;
      spinning_ = false;
      active_ = false;
      RCLCPP_INFO(node_->get_logger(), "[Pose Sharing] deactivate");

      for (auto &pair : active_bridges) {
          pair.second->remove_from_executor(executor);
          pair.second.reset();
      }
      active_bridges.clear();
  }

  void spin_executor()
  {
    if (!spinning_) return;

    executor.spin_some();
  }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    bool spinning_ = false;
};

#endif  // COMMUNICATION_MANAGER__POSE_SHARING_HPP_