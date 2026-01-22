#ifndef HV_HANDLER__HV_BASE_HPP_
#define HV_HANDLER__HV_BASE_HPP_

#include <fstream>

#include <rclcpp/rclcpp.hpp>
#include <nlohmann/json.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

using json = nlohmann::json;
using namespace std::chrono_literals;
using real = double;

class HVBase {
public:
    HVBase() = default;
    virtual ~HVBase() = default;

    struct Path
    {
        std::vector<real> X;
        std::vector<real> Y;
        size_t N = 0;
    };
    
    struct Vehicle
    {
        std::string name = "";
        int start_index = 0;
        const Path* path = nullptr;

        real v = 0.75;
        real Ld = 0.3;

        real x = 0.0;
        real y = 0.0;
        real yaw = 0.0;

        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub;
    };
    
    virtual void activate() = 0;
    virtual void deactivate() = 0;
    bool is_active() const { return active_; }

    Vehicle create_vehicle(int id, const Path &path, int start_idx, real Ld=0.3, real init_v=0.75)
    {
        Vehicle v;
        v.name = "HV_" + std::to_string(id);
        v.path = &path;
        v.Ld = Ld;
        v.v = init_v;

        int N = static_cast<int>(path.N);
        v.start_index = std::max(0, std::min(start_idx, N - 2));

        v.x = path.X[v.start_index];
        v.y = path.Y[v.start_index];

        v.yaw = std::atan2(
            path.Y[v.start_index + 1] - path.Y[v.start_index],
            path.X[v.start_index + 1] - path.X[v.start_index]);

        v.pub = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/" + v.name, 10);

        // RCLCPP_INFO(node_->get_logger(),
        //     "Init %s at (x=%.3f,y=%.3f,yaw=%.2fdeg)",
        //     v.name.c_str(), v.x, v.y, v.yaw * 180.0 / M_PI);

        return v;
    }

    real compute_yaw_rate(const Vehicle &veh)
    {
        int closest_i = 0;
        real min_dist = 1e9;

        size_t N_ = veh.path->N;
        real v_ = veh.v;
        real Ld_ = veh.Ld;

        for (size_t i = 0; i < N_; i++) {
            real dx = veh.path->X[i] - veh.x;
            real dy = veh.path->Y[i] - veh.y;
            real dist = dx*dx + dy*dy;

            if (dist < min_dist) {
                min_dist = dist;
                closest_i = i;
            }
        }

        int target_i = closest_i;
        real accum_dist = 0.0;

        for (int k = 0; k < N_; k++) {
            int i = (closest_i + k) % N_;

            real dx = veh.path->X[i] - veh.x;
            real dy = veh.path->Y[i] - veh.y;
            real dist = std::sqrt(dx*dx + dy*dy);

            if (dist > Ld_) {
                target_i = i;
                break;
            }
        }

        real tx = veh.path->X[target_i];
        real ty = veh.path->Y[target_i];

        // 차량 기준 좌표계 변환
        real dx = tx - veh.x;
        real dy = ty - veh.y;

        real fx =  std::cos(veh.yaw) * dx + std::sin(veh.yaw) * dy;
        real fy = -std::sin(veh.yaw) * dx + std::cos(veh.yaw) * dy;

        real curvature = 2.0 * fy / (Ld_ * Ld_);
        real omega = v_ * curvature;

        // saturation
        if (omega > 2.0)  omega = 2.0;
        if (omega < -2.0) omega = -2.0;

        return omega;
    }

    void rk4_step(real &x, real &y, real &yaw, real v, real omega, real dt_=0.05)
    {
        auto f = [&](real Yaw, real &dx, real &dy, real &dyaw)
        {
            dx = v * std::cos(Yaw);
            dy = v * std::sin(Yaw);
            dyaw = omega;
        };

        real k1x, k1y, k1yaw;
        real k2x, k2y, k2yaw;
        real k3x, k3y, k3yaw;
        real k4x, k4y, k4yaw;

        f(yaw, k1x, k1y, k1yaw);
        f(yaw + 0.5*dt_*k1yaw, k2x, k2y, k2yaw);
        f(yaw + 0.5*dt_*k2yaw, k3x, k3y, k3yaw);
        f(yaw + dt_*k3yaw,     k4x, k4y, k4yaw);

        real w = dt_ / 6.0;

        x   += w * (k1x + 2*k2x + 2*k3x + k4x);
        y   += w * (k1y + 2*k2y + 2*k3y + k4y);
        yaw += w * (k1yaw + 2*k2yaw + 2*k3yaw + k4yaw);

        while (yaw >  M_PI) yaw -= 2*M_PI;
        while (yaw < -M_PI) yaw += 2*M_PI;
    }


    virtual void set_pause(bool pause_){
        is_pause = pause_;
    }

protected:
    bool active_ = false;
    rclcpp::Node::SharedPtr node_;
    bool is_pause = false;
};

#endif