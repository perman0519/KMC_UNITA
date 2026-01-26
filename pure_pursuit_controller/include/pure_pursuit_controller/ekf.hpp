#ifndef EKF_HPP
#define EKF_HPP

#include <Eigen/Dense>
#include <cmath>

class VelocityEKF {
public:
    VelocityEKF() {
        is_initialized = false;
        Q.setIdentity(); Q *= 0.01;
        R.setIdentity(); R *= 0.05;
        P.setIdentity();
    }

    void init(double x, double y, double v, double theta) {
        state << x, y, v, theta;
        is_initialized = true;
    }

    void predict(double dt) {
        double v = state(2); double theta = state(3);
        state(0) += v * std::cos(theta) * dt;
        state(1) += v * std::sin(theta) * dt;
        Eigen::Matrix4d F = Eigen::Matrix4d::Identity();
        F(0, 2) = std::cos(theta) * dt; F(0, 3) = -v * std::sin(theta) * dt;
        F(1, 2) = std::sin(theta) * dt; F(1, 3) = v * std::cos(theta) * dt;
        P = F * P * F.transpose() + Q;
    }

    void update(double z_x, double z_y) {
        Eigen::Vector2d z(z_x, z_y);
        Eigen::Matrix<double, 2, 4> H; H.setZero(); H(0, 0) = 1.0; H(1, 1) = 1.0;
        Eigen::Vector2d y = z - H * state;
        Eigen::Matrix2d S = H * P * H.transpose() + R;
        Eigen::Matrix<double, 4, 2> K = P * H.transpose() * S.inverse();
        state = state + K * y;
        P = (Eigen::Matrix4d::Identity() - K * H) * P;
    }

    bool is_initialized;
    Eigen::Vector4d state; // [x, y, v, theta]
    Eigen::Matrix4d P, Q;
    Eigen::Matrix2d R;
};

#endif
