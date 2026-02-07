#ifndef COMMUNICATION_MANAGER__COMMUNICATION_BASE_HPP_
#define COMMUNICATION_MANAGER__COMMUNICATION_BASE_HPP_

#include <chrono>
#include <iomanip>
#include <sstream>
#include <utility> 

#include <rclcpp/rclcpp.hpp>
#include <domain_bridge/domain_bridge.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/accel.hpp>

using namespace std::chrono_literals;
using real = double;

class CommunicationBase
{
public:
    virtual ~CommunicationBase() = default;

    virtual void activate() = 0;
    virtual void deactivate() = 0;
    bool is_active() const { return active_; }

    struct TopicInfo
    {
        std::string name;
        int id;
        bool is_cav;
    };
    
    TopicInfo create_topic_info(const std::string &topic_name, bool is_cav)
    {
        TopicInfo info;
        info.name = topic_name;
        info.is_cav = is_cav;
        if (is_cav) {
            info.id = std::stoi(topic_name.substr(5, 2));
        } else {
            info.id = std::stoi(topic_name.substr(4, 2));
        }
        return info;
    }

    void vec_topics_init()
    {
        RCLCPP_INFO(node_->get_logger(), "Wait for 1s...");
        rclcpp::sleep_for(std::chrono::seconds(1));

        vec_topics.clear();

        int CAV_N = 0;
        int HV_N = 0;

        auto topics = node_->get_topic_names_and_types();
        for (auto it = topics.begin(); it != topics.end(); ) 
        {
            const std::string& topic_name = it->first;
            const std::vector<std::string> &topic_types = it->second;

            if (topic_name.find("/CAV_", 0) == 0 && 
                topic_name.size() == 7 && 
                topic_types[0] == "geometry_msgs/msg/PoseStamped") 
            {
            vec_topics.push_back(create_topic_info(topic_name, true));
            ++it;
            CAV_N++;
            continue;
            }
            if (topic_name.find("/HV", 0) == 0 && 
                topic_name.size() == 6 && 
                topic_types[0] == "geometry_msgs/msg/PoseStamped") 
            {
            vec_topics.push_back(create_topic_info(topic_name, false));
            ++it;
            HV_N++;
            continue;
            }
            ++it;
        }

        // RCLCPP_INFO(node_->get_logger(), "Subscribe %d CAVs' Topic and %d HVs' Topic!!", CAV_N, HV_N);
    }

    void pose_communication()
    {
        for (const auto &topic_info : vec_topics)
        {
            if (topic_info.is_cav)
            {
                std::ostringstream node_name_ss;
                node_name_ss << "f100t" << std::setw(2) << std::setfill('0') << topic_info.id;

                domain_bridge::DomainBridgeOptions bridge_opts;
                bridge_opts.name(node_name_ss.str());

                auto db_node = std::make_shared<domain_bridge::DomainBridge>(bridge_opts);

                // RCLCPP_INFO(node_->get_logger(),
                //             "Transmitting CAV pose and control cmd between Domain 100 and #%d", topic_info.id);
                std::ostringstream f_topic_name_pose;
                f_topic_name_pose << "Ego_pose";
                domain_bridge::TopicBridgeOptions topic_option_pose;
                topic_option_pose.remap_name(f_topic_name_pose.str());
                
                std::ostringstream f_topic_name_accel;
                f_topic_name_accel << "CAV_" << std::setw(2) << std::setfill('0') << topic_info.id << "_accel";
                domain_bridge::TopicBridgeOptions topic_option_accel;
                topic_option_accel.remap_name(f_topic_name_accel.str());

                db_node->bridge_topic(topic_info.name, "geometry_msgs/msg/PoseStamped", 100, topic_info.id, topic_option_pose);    
                db_node->bridge_topic("/Accel", "geometry_msgs/msg/Accel", topic_info.id, 100, topic_option_accel);    
                
                std::pair<int, int> p = std::make_pair(100, topic_info.id);
                active_bridges[p] = db_node;
                db_node->add_to_executor(executor);
            }

            else
            {
                for (const auto &cav_topic : vec_topics)
                {
                    if (!cav_topic.is_cav) {
                        continue;
                    }
                    std::ostringstream node_name_ss;
                    node_name_ss << "f"<< std::setw(2) << std::setfill('0') << topic_info.id <<"t" << std::setw(2) << std::setfill('0') << cav_topic.id;

                    domain_bridge::DomainBridgeOptions bridge_opts;
                    bridge_opts.name(node_name_ss.str());

                    auto db_node = std::make_shared<domain_bridge::DomainBridge>(bridge_opts);

                    // RCLCPP_INFO(node_->get_logger(),
                    //             "Transmitting HV %d pose to CAV %d's Domain", topic_info.id, cav_topic.id);
                    
                    db_node->bridge_topic(topic_info.name, "geometry_msgs/msg/PoseStamped", 100, cav_topic.id);

                    std::pair<int, int> p = std::make_pair(topic_info.id, cav_topic.id);
                    active_bridges[p] = db_node;
                    db_node->add_to_executor(executor);
                }
            }
        }
    }

protected:
    bool active_ = false;
    std::vector<TopicInfo> vec_topics;
    rclcpp::Node::SharedPtr node_;

    std::map<std::pair<int, int>, std::shared_ptr<domain_bridge::DomainBridge>> active_bridges;
    rclcpp::executors::SingleThreadedExecutor executor;
};

#endif  // COMMUNICATION_MANAGER__COMMUNICATION_BASE_HPP_