#pragma once
#include "hv_base.hpp"

#include <chrono>
#include <fstream>
#include <vector>
#include <cmath>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;

class HVPromblemTwo : public HVBase {
public:
    HVPromblemTwo(const rclcpp::Node::SharedPtr& node){
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

        // default values
        lane3_init_v_ = 0.5;
        lane2_init_v_ = 0.75;
        lane3_ld_     = 0.3;
        lane2_ld_     = 0.3;

        if (config["problem_two"]) {
            auto p2 = config["problem_two"];

            if (p2["lane_three"]) {
                lane3_init_v_ =
                    p2["lane_three"]["init_speed"].as<double>(lane3_init_v_);
                lane3_ld_ =
                    p2["lane_three"]["lookahead"].as<double>(lane3_ld_);
            }

            if (p2["lane_two"]) {
                lane2_init_v_ = 
                    p2["lane_two"]["init_speed"].as<double>(lane2_init_v_);
                lane2_ld_ =
                    p2["lane_two"]["lookahead"].as<double>(lane2_ld_);
            }
        }

        // RCLCPP_INFO(
        //     node_->get_logger(),
        //     "[Problem2] lane3 v=%.2f ld=%.2f | lane2 v=%.2f ld=%.2f",
        //     lane3_init_v_, lane3_ld_,
        //     lane2_init_v_, lane2_ld_
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

        load_json(pkg_path + "/config/belt_lane_one.json",
                lane_one_json, "belt lane one");
        load_json(pkg_path + "/config/belt_lane_two.json",
                lane_two_json, "belt lane two");
        load_json(pkg_path + "/config/belt_lane_three.json",
                lane_three_json, "belt lane three");

        lane_one_path.X = lane_one_json["X"].get<std::vector<real>>();
        lane_one_path.Y = lane_one_json["Y"].get<std::vector<real>>();
        lane_one_path.N = lane_one_path.X.size();

        lane_two_path.X = lane_two_json["X"].get<std::vector<real>>();
        lane_two_path.Y = lane_two_json["Y"].get<std::vector<real>>();
        lane_two_path.N = lane_two_path.X.size();

        lane_three_path.X = lane_three_json["X"].get<std::vector<real>>();
        lane_three_path.Y = lane_three_json["Y"].get<std::vector<real>>();
        lane_three_path.N = lane_three_path.X.size();
    }

    void pose_initialize() {
        // idx, path, start_idx, Ld, init_v
        int idx = 0;
        for (int id = 19; id <= 22; ++id) {
            vehicles_.push_back(create_vehicle(id, lane_three_path, idx, lane3_ld_, lane3_init_v_));
            idx += 650;  
        }

        idx = 0;
        for (int id = 23; id <= 30; ++id) {
            vehicles_.push_back(create_vehicle(id, lane_two_path, idx, lane2_ld_, lane2_init_v_));
            idx += 370;  
        }

        idx = 0;
        for (int id = 31; id <= 36; ++id) {
            vehicles_.push_back(create_vehicle(id, lane_one_path, idx, 0.3, 0.0));
            idx += 530;  
        }
    }

    void activate() override {
        if (active_) return;
        active_ = true;
        RCLCPP_INFO(node_->get_logger(), "[Problem2] activate");
        pose_initialize();

        timer_ = node_->create_wall_timer(50ms, std::bind(&HVPromblemTwo::timerCallback, this));

        // RCLCPP_INFO(node_->get_logger(),
        //     "Multi-vehicle HV follower started: HV_19~22 on lane3, HV_23~30 on lane2");
    }

    void deactivate() override {
        if (!active_) return;
        active_ = false;
        RCLCPP_INFO(node_->get_logger(), "[Problem2] deactivate");

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
    json lane_one_json;
    json lane_two_json;
    json lane_three_json;

    Path lane_one_path;
    Path lane_two_path;
    Path lane_three_path;

    std::vector<Vehicle> vehicles_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    double lane3_init_v_;
    double lane2_init_v_;
    double lane3_ld_;
    double lane2_ld_;
};
