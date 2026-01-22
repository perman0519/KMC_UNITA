#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "communication_manager/pose_sharing.hpp"

#include "scene_srv/srv/start_signal.hpp"
#include "scene_srv/srv/scene_signal.hpp"

class PoseSharingHandler : public rclcpp::Node {
public:
    static std::shared_ptr<PoseSharingHandler> create() {
        auto node = std::shared_ptr<PoseSharingHandler>(new PoseSharingHandler());
        node->postInit();   // shared_ptr 생성 이후
        return node;
    }

    PoseSharingHandler() : Node("pose_sharing_node") {
        server_ = this->create_service<scene_srv::srv::StartSignal>(
            "start_signal",
            std::bind(
                &PoseSharingHandler::onRequest,
                this,
                std::placeholders::_1,
                std::placeholders::_2
            )
        );

        client_scene = this->create_client<scene_srv::srv::SceneSignal>("scene_signal");
    }

private:
    void postInit() {
        pose_sharing_ =
            std::make_shared<PoseSharing>(shared_from_this());
        RCLCPP_INFO(this->get_logger(), "Pose Sharing Node initialized");
    }

    void onRequest(
        const std::shared_ptr<scene_srv::srv::StartSignal::Request> req,
        std::shared_ptr<scene_srv::srv::StartSignal::Response> res
    ) {
        int sig = req->new_event;

        switch (sig)
        {
        case 0:
            deactivateAll();
            res->success = true;
            break;
        
        case 1:
            pose_sharing_->activate();
            res->success = true;
        
        default:
            res->success = true;
            break;
        }

        // std::cout << "Requesting scene change to: " << req->scene_number << std::endl;
        if (!client_scene->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Service not available");
            return;
        }
        auto scene_req = std::make_shared<scene_srv::srv::SceneSignal::Request>();
        scene_req->signal = req->scene_number;
        scene_req->state = sig;

        client_scene->async_send_request(
        scene_req,
        [this](rclcpp::Client<scene_srv::srv::SceneSignal>::SharedFuture future) {
            try {
                auto scene_res = future.get();
                // RCLCPP_INFO(
                //     this->get_logger(),
                //     "Scene service response: %s",
                //     scene_res->message.c_str()
                // );
            } catch (const std::exception &e) {
                // RCLCPP_ERROR(
                //     this->get_logger(),
                //     "Scene service exception: %s",
                //     e.what()
                // );
            }
        });
    }

    void deactivateAll() {
        if (pose_sharing_->is_active()) pose_sharing_->deactivate();
    }

private:
    rclcpp::Service<scene_srv::srv::StartSignal>::SharedPtr server_;
    rclcpp::Client<scene_srv::srv::SceneSignal>::SharedPtr client_scene;
    std::shared_ptr<PoseSharing> pose_sharing_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
        
    auto node = PoseSharingHandler::create();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
