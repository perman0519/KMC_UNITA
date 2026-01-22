#include <rclcpp/rclcpp.hpp>
#include "hv_handler/hv_problem_two.hpp"
#include "hv_handler/hv_problem_three.hpp"
#include "scene_srv/srv/scene_signal.hpp"

class HV_Handler : public rclcpp::Node {
public:
    static std::shared_ptr<HV_Handler> create() {
        auto node = std::shared_ptr<HV_Handler>(new HV_Handler());
        node->postInit();   // shared_ptr 생성 이후
        return node;
    }

    HV_Handler() : Node("hv_handler_node") {
        server_ = this->create_service<scene_srv::srv::SceneSignal>(
            "scene_signal",
            std::bind(
                &HV_Handler::onRequest,
                this,
                std::placeholders::_1,
                std::placeholders::_2
            )
        );
    }

private:
    void postInit() {
        problemtwo_ =
            std::make_shared<HVPromblemTwo>(shared_from_this());
        RCLCPP_INFO(this->get_logger(), "Problem #2 Node initialized");

        problemthree_ =
            std::make_shared<HVPromblemThree>(shared_from_this());
        RCLCPP_INFO(this->get_logger(), "Problem #3 Node initialized");
    }

    void onRequest(
        const std::shared_ptr<scene_srv::srv::SceneSignal::Request> req,
        std::shared_ptr<scene_srv::srv::SceneSignal::Response> res
    ) {
        int64_t sig = req->signal;
        int64_t state = req->state;

        switch (state)
        {
        case 0:
            deactivateAll();
            return;

        case 1:
            if (sig == 20) 
                problemtwo_->set_pause(false);
            else if (sig == 30) 
                problemthree_->set_pause(false);
            break;
        
        default:
            if (sig == 20) 
                problemtwo_->set_pause(true);
            else if (sig == 30) 
                problemthree_->set_pause(true);
            break;
        }

        switch (sig) {
            case 20:
                problemtwo_->activate();
                res->success = true;
                res->message = "Problem #2 activated";
                break;

            case 30:
                problemthree_->activate();
                res->success = true;
                res->message = "Problem #3 activated";
                break;

            default:
                res->success = true;
                res->message = "All HV removed";
                break;
        }
    }

    void deactivateAll() {
        if (problemtwo_->is_active()) {
            // problemtwo_->pose_initialize();
            problemtwo_->deactivate();
        }
        if (problemthree_->is_active()) {
            // problemthree_->pose_initialize();
            problemthree_->deactivate();
        }
    }

private:
    rclcpp::Service<scene_srv::srv::SceneSignal>::SharedPtr server_;
    std::shared_ptr<HVPromblemTwo> problemtwo_;
    std::shared_ptr<HVPromblemThree> problemthree_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
        
    auto node = HV_Handler::create();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
