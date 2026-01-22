#pragma once
#include "hv_base.hpp"

#include <fstream>
#include <vector>
#include <cmath>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

class HVPromblemThree : public HVBase {
public:
    HVPromblemThree(const rclcpp::Node::SharedPtr& node) 
    {
        node_ = node;

        std::string pkg_path = ament_index_cpp::get_package_share_directory("hv_handler");

        std::string yaml_path = pkg_path + "/config/hv_params.yaml";

        YAML::Node config;
        try {
            config = YAML::LoadFile(yaml_path);
            // RCLCPP_INFO(
            //     node_->get_logger(),
            //     "Loaded HV params from %s",
            //     yaml_path.c_str()
            // );
        } catch (const std::exception& e) {
            RCLCPP_WARN(
                node_->get_logger(),
                "Failed to load YAML (%s), using defaults",
                e.what()
            );
        }
        init_v_ = 0.75;
        ld_     = 0.3;

        if (config && config["problem_three"] &&
            config["problem_three"]["roundabout"])
        {
            auto p3 = config["problem_three"]["roundabout"];
    
            init_v_ = p3["init_speed"].as<double>(init_v_);
            ld_     = p3["lookahead"].as<double>(ld_);
        }

        // RCLCPP_INFO(
        //     node_->get_logger(),
        //     "[Problem3] v=%.2f ld=%.2f",
        //     init_v_, ld_
        // );

        auto load_json = [&](const std::string &path, json &j, const char *name)
        {
            std::ifstream f(path);
            if (!f.is_open()) {
                throw std::runtime_error(
                    std::string("Failed to open ") + name + ": " + path);
            }
            // std::cout<<"Loading "<< name << " from " << path << std::endl;
            f >> j;
        };

        load_json(pkg_path + "/config/roundabout_lane_two.json",
                roundabout_json, "roundabout lane two");

        roundabout_path.X = roundabout_json["X"].get<std::vector<real>>();
        roundabout_path.Y = roundabout_json["Y"].get<std::vector<real>>();
        roundabout_path.N = roundabout_path.X.size();
    }

    void pose_initialize() {
        // idx, path, start_idx, Ld, init_v
        vehicles_.push_back(create_vehicle(19, roundabout_path, 0,   ld_, init_v_));      
        vehicles_.push_back(create_vehicle(20, roundabout_path, 300, ld_, init_v_));   
    }

    void activate() override {
        if (active_) return;
        active_ = true;
        RCLCPP_INFO(node_->get_logger(), "[Problem3] activate");

        pose_initialize();
        timer_ = node_->create_wall_timer(50ms, std::bind(&HVPromblemThree::timerCallback, this));

        // RCLCPP_INFO(node_->get_logger(),
        //     "Multi-vehicle HV follower started: HV_19 & HV_20");
    }

    void deactivate() override {
        if (!active_) return;
        active_ = false;
        RCLCPP_INFO(node_->get_logger(), "[Problem3] deactivate");

        timer_.reset();

        for(auto &veh : vehicles_) {
            veh.pub.reset();
        }
        vehicles_.clear();
    }

private:
    void timerCallback()
    {
        if (is_pause)
            return;
        for (auto &veh : vehicles_)
        {
            real omega = compute_yaw_rate(veh);
            rk4_step(veh.x, veh.y, veh.yaw, veh.v, omega, 0.05);

            geometry_msgs::msg::PoseStamped msg;
            msg.header.stamp = node_->now();
            msg.header.frame_id = "map";

            msg.pose.position.x = veh.x;
            msg.pose.position.y = veh.y;
            msg.pose.position.z = 0.13f;

            msg.pose.orientation.x = 1.57f;
            msg.pose.orientation.y = 0.0f;
            msg.pose.orientation.z = veh.yaw;
            msg.pose.orientation.w = 1.0f;

            veh.pub->publish(msg);
        }
    }

private:
    json roundabout_json;
    Path roundabout_path;

    double init_v_;
    double ld_;

    std::vector<Vehicle> vehicles_;
    rclcpp::TimerBase::SharedPtr timer_;
};