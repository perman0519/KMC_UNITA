#pragma once

#include "pch.hpp"
#include "scene_srv/srv/scene_signal.hpp"
#include "scene_srv/srv/start_signal.hpp"
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int64.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/accel.hpp>

using namespace std::chrono_literals;

struct InputData {
    real linear_velocity;
    real angular_velocity;

    real position_x;
    real position_y;
    real yaw;

    bool is_cav;
    bool received = false;
};

struct OutputData {
    real position_x;
    real position_y;
    real yaw;
};

struct TopicName {
    std::string pub;
    std::string sub;
    int id;
    bool is_cav;
};

class ROSNode : public rclcpp::Node {
private:
    std::unordered_map<std::string, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> publisher;
    std::unordered_map<std::string, rclcpp::Subscription<geometry_msgs::msg::Accel>::SharedPtr> subscriber_cav;
    std::unordered_map<std::string, rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> subscriber_hv;
    std::unordered_map<std::string, geometry_msgs::msg::Accel::SharedPtr> messages_cav;
    std::unordered_map<std::string, geometry_msgs::msg::PoseStamped::SharedPtr> messages_hv;
    std::unordered_map<std::string, int> AVIDs;
    std::unordered_map<std::string, int> HVIDs;
    std::deque<int> AVList;
    std::deque<int> HVList;

    rclcpp::Client<scene_srv::srv::StartSignal>::SharedPtr client_start;
    std::unordered_map<std::string, OutputData> published_messages;

    std::unordered_map<int, int> profileToSceneMap;
    int prevState = -1;
    int saved_sceneNumber = -1;

    rclcpp::TimerBase::SharedPtr timer_;

public:
    ROSNode() : Node("testbed_simulation") {
        reset();
        // scenePublisher = this->create_publisher<std_msgs::msg::Int64>("scene_update", 10);
        // client_scene = this->create_client<scene_srv::srv::SceneSignal>("scene_signal");
        client_start = this->create_client<scene_srv::srv::StartSignal>("start_signal");

        profileToSceneMap.insert({1, 11});
        profileToSceneMap.insert({2, 12});
        profileToSceneMap.insert({3, 20});
        profileToSceneMap.insert({4, 30});

        timer_ = this->create_wall_timer(50ms, std::bind(&ROSNode::timerCallback, this));
    }

    std::vector<std::string> discoverTopics(const std::string &pattern = "vehicle") {
        auto topic_names_and_types = this->get_topic_names_and_types();
        std::vector<std::string> matching_topics;

        for (const auto &[topic_name, types] : topic_names_and_types) {
            if (topic_name.find(pattern) != std::string::npos) {
                matching_topics.push_back(topic_name);
            }
        }
        return matching_topics;
    }

    TopicName createEntity(bool AV) {
        int number;
        std::string type;
        TopicName name;

        if (AV) {
            number = AVList.front();
            type = "CAV";
            AVList.pop_front();
        } else {
            number = HVList.front();
            type = "HV";
            HVList.pop_front();
        }
        std::ostringstream oss;
        oss << std::setw(2) << std::setfill('0') << number;
        std::string padded = oss.str();
        name.id = number;

        auto qos = rclcpp::SensorDataQoS();

        if (AV) {
            name.sub = "/" + type + "_" + padded + "_accel";
            name.pub = "/" + type + "_" + padded;
            name.is_cav = true;
            AVIDs[name.sub] = number;

            subscriber_cav[name.sub] = this->create_subscription<geometry_msgs::msg::Accel>(
                name.sub, qos, [this, name](const geometry_msgs::msg::Accel::SharedPtr msg) { messages_cav[name.sub] = msg; });
            publisher[name.pub] = this->create_publisher<geometry_msgs::msg::PoseStamped>(name.pub, qos);
        } else {
            name.sub = "/" + type + "_" + padded;
            name.is_cav = false;
            HVIDs[name.sub] = number;

            subscriber_hv[name.sub] = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                name.sub, qos, [this, name](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { messages_hv[name.sub] = msg; });
        }
        return name;
    }

    void removeEntity(TopicName &name, bool AV) {
        if (AV) {
            AVList.push_front(AVIDs[name.sub]);
            subscriber_cav[name.sub].reset();
            publisher[name.pub].reset();
        } else {
            HVList.push_front(HVIDs[name.sub]);
            subscriber_hv[name.sub].reset();
        }
    }

    InputData receiveTopic(const std::string &subName) {
        InputData data;
        auto it = messages_cav.find(subName);
        if (it != messages_cav.end() && it->second != nullptr) {
            data.linear_velocity = it->second->linear.x;
            data.angular_velocity = it->second->angular.z;

            data.position_x = 0.0f;
            data.position_y = 0.0f;
            data.yaw = 0.0f;

            data.is_cav = true;
            data.received = true;
        }

        auto it_ = messages_hv.find(subName);
        if (it_ != messages_hv.end() && it_->second != nullptr) {
            data.linear_velocity = 0.0f;
            data.angular_velocity = 0.0f;

            data.position_x = it_->second->pose.position.x;
            data.position_y = it_->second->pose.position.y;
            data.yaw = it_->second->pose.orientation.z;

            data.is_cav = false;
            data.received = true;
        }

        return data;
    }

    void timerCallback() {
        for (const auto &[pub, data] : published_messages) {
            rclcpp::Time sync_time = this->now();
            // prev_time = sync_time;

            auto message = geometry_msgs::msg::PoseStamped();
            message.header.stamp = sync_time;
            message.header.frame_id = "world";
            message.pose.position.x = data.position_x;
            message.pose.position.y = data.position_y;
            message.pose.position.z = 0.13f;
            message.pose.orientation.x = 1.57f;
            message.pose.orientation.y = 0.0f;
            message.pose.orientation.z = data.yaw;
            message.pose.orientation.w = 1.0f;
            publisher[pub]->publish(message);
        }
    }

    void sendTopic(const std::string &pubName, OutputData &data) {

        auto it = publisher.find(pubName);
        if (it == publisher.end() || !it->second) {
            std::cerr << "Publisher for " << pubName << " not found or not initialized!" << std::endl;
            return;
        }

        published_messages[pubName] = data;
        // publisher[pubName]->publish(message);
    }

    void changeScene(int sceneNumber) {
        saved_sceneNumber = profileToSceneMap.find(sceneNumber) != profileToSceneMap.end() ? profileToSceneMap[sceneNumber] : 100;

        // auto req = std::make_shared<scene_srv::srv::SceneSignal::Request>();
        // req->signal = profileToSceneMap.find(sceneNumber) != profileToSceneMap.end() ? profileToSceneMap[sceneNumber] : 100;

        // auto future = client_scene->async_send_request(req);
        // auto ret = rclcpp::spin_until_future_complete(
        //     shared_from_this(),
        //     future);

        // if (ret == rclcpp::FutureReturnCode::SUCCESS) {
        //     auto res = future.get();
        //     RCLCPP_INFO(this->get_logger(), "Response: %s", res->message.c_str());
        // } else {
        //     RCLCPP_ERROR(this->get_logger(), "Service call failed");
        // }
    }

    void changeState(int stateNumber) {
        if (!client_start->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Communication not available");
            return;
        }

        auto req = std::make_shared<scene_srv::srv::StartSignal::Request>();

        req->scene_number = saved_sceneNumber;
        req->new_event = stateNumber;
        // if (prevState == 0 && stateNumber == 1) {
        //     req->new_event = true;
        //     prevState = stateNumber;
        // } else {
        //     req->new_event = false;
        //     prevState = stateNumber;
        //     return;
        // }

        auto future = client_start->async_send_request(req);
        auto ret = rclcpp::spin_until_future_complete(
            shared_from_this(),
            future);

        if (ret == rclcpp::FutureReturnCode::SUCCESS) {
            auto res = future.get();

            if (res->success)
                RCLCPP_INFO(this->get_logger(), "Communication Function Successfully Worked!");
            else
                RCLCPP_WARN(this->get_logger(), "Communication Function Failed!");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Communication call failed");
        }
    }

    void reset() {
        for (auto &[name, pub] : publisher)
            pub.reset();
        for (auto &[name, sub] : subscriber_cav)
            sub.reset();
        for (auto &[name, sub] : subscriber_hv)
            sub.reset();
        publisher.clear();
        published_messages.clear();
        subscriber_cav.clear();
        subscriber_hv.clear();
        messages_cav.clear();
        messages_hv.clear();
        AVIDs.clear();
        HVIDs.clear();
        AVList.clear();
        HVList.clear();

        for (size_t i = 1; i < 19; ++i) {
            AVList.push_back(i);
            HVList.push_back(i + 18);
        }
    }
};