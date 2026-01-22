#include "engine.hpp"
#include <rclcpp/rclcpp.hpp>

extern Engine *createApp();

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    Engine *engine = createApp();
    engine->run();

    rclcpp::shutdown();
    delete engine;
}
// to test run the program and input the following:
// ros2 topic pub /vehicle_1/input geometry_msgs/msg/Vector3 "{x: 1.0, y: 0.05, z: 0.1}" --rate 10
