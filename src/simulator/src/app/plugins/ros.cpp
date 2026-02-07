
// #include <rclcpp/rclcpp.hpp>
// #include <std_msgs/msg/string.hpp>
// #include "my_custom_msgs_mc/msg/custom_message.hpp"

#include "pch.hpp"
#include "engine/engine.hpp"
#include "app/plugins.hpp"

namespace ROSSystem {

    void init(ECS &ecs, World &world, real dt) {
    }

    void rk4_step_pose(real x, real y, real yaw, real v, real omega, real dt,
                       real &x_next, real &y_next, real &yaw_next) {
        auto f = [&](real /*X*/, real /*Y*/, real Yaw, real &dX, real &dY, real &dYaw) {
            dX = v * std::cos(Yaw);
            dY = v * std::sin(Yaw);
            dYaw = omega;
        };

        real k1x, k1y, k1yaw;
        real k2x, k2y, k2yaw;
        real k3x, k3y, k3yaw;
        real k4x, k4y, k4yaw;

        f(x, y, yaw, k1x, k1y, k1yaw);
        f(x + 0.5f * dt * k1x, y + 0.5f * dt * k1y, yaw + 0.5f * dt * k1yaw, k2x, k2y, k2yaw);
        f(x + 0.5f * dt * k2x, y + 0.5f * dt * k2y, yaw + 0.5f * dt * k2yaw, k3x, k3y, k3yaw);
        f(x + dt * k3x, y + dt * k3y, yaw + dt * k3yaw, k4x, k4y, k4yaw);

        const real w = dt / 6.0f;
        x_next = x + w * (k1x + 2.0f * k2x + 2.0f * k3x + k4x);
        y_next = y + w * (k1y + 2.0f * k2y + 2.0f * k3y + k4y);
        yaw_next = yaw + w * (k1yaw + 2.0f * k2yaw + 2.0f * k3yaw + k4yaw);
    }

    void receiveData(ECS &ecs, World &world, real dt) {
        auto view = ecs.write<Vehicle, State, ROSTopic, ROSData, Physics>();
        view.iterate([&](EntityID id, Vehicle &vehicle, State &state, ROSTopic &topic, ROSData &data, Physics &physics) {
            if (!physics.on)
                return;

            // newData: geometry_msgs::msg::AccelStamped
            InputData newData = world.ros->receiveTopic(topic.subscriber);
            if (newData.is_cav) {
                if (newData.received) {
                    data.linear_velocity = newData.linear_velocity;
                    data.angular_velocity = newData.angular_velocity;
                    data.updated = true;
                    ecs.markChanged<ROSData>(id);
                    newData.received = false;
                }

                real yaw_temp = glm::eulerAngles(state.rotation).z;
                rk4_step_pose(state.position.x, state.position.y, yaw_temp,
                              data.linear_velocity, data.angular_velocity, dt,
                              state.position.x, state.position.y, yaw_temp);
                state.rotation = glm::angleAxis(static_cast<float>(yaw_temp), vec3(0, 0, 1));
                ecs.markChanged<State>(id);
            } else {
                if (newData.received) {
                    state.position.x = newData.position_x;
                    state.position.y = newData.position_y;
                    state.rotation = glm::angleAxis(static_cast<float>(newData.yaw), vec3(0, 0, 1));
                    data.updated = true;
                    ecs.markChanged<ROSData>(id);
                    ecs.markChanged<State>(id);
                    newData.received = false;

                    // if (topic.subscriber.find("19") != std::string::npos) {
                    //     std::cout<<"[LOG] HV19 received pose update from ROS2: {"
                    //              << "x: " << state.position.x
                    //              << ", y: " << state.position.y
                    //              << "}\n";
                    // }
                }
            }
        });
    }

    void sendData(ECS &ecs, World &world, real dt) {
        auto view = ecs.write<Vehicle, State, Motion, ROSTopic, Physics>();
        view.iterate([&](EntityID id, Vehicle &vehicle, State &state, Motion &motion, ROSTopic &topic, Physics &physics) {
            if (!physics.on)
                return;

            if (vehicle.AV == false)
                return;

            OutputData data;
            data.position_x = state.position.x;
            data.position_y = state.position.y;
            real yaw = glm::eulerAngles(state.rotation).z;
            data.yaw = yaw;
            world.ros->sendTopic(topic.publisher, data);
        });
    }

}; // namespace ROSSystem

class Engine;
void ROSPlusgin(Engine &engine) {
    engine.addLogicSystem(SystemType::INIT, ROSSystem::init);
    engine.addLogicSystem(SystemType::UPDATE, ROSSystem::receiveData);
    engine.addLogicSystem(SystemType::UPDATE, ROSSystem::sendData);
};
