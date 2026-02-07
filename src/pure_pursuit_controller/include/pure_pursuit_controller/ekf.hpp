#ifndef EKF_HPP
#define EKF_HPP

#include <Eigen/Dense>
#include <cmath>

class VelocityEKF {
public:
    VelocityEKF() {
        is_initialized = false;
        Q.setIdentity(); Q *= 0.01;  // 실내 환경, 외란 적음

        // 모션 캡처: 매우 정확 (mm 단위)
        R3.setIdentity();
        R3(0, 0) = 0.001;  // x 위치: 매우 정확
        R3(1, 1) = 0.001;  // y 위치: 매우 정확
        R3(2, 2) = 0.005;  // yaw: 위치보다 약간 노이즈

        // 모션 캡처 + 엔코더
        R4.setIdentity();
        R4(0, 0) = 0.001;  // x 위치: 매우 정확
        R4(1, 1) = 0.001;  // y 위치: 매우 정확
        R4(2, 2) = 0.02;   // v 속도: 엔코더 (미끄러짐 가능)
        R4(3, 3) = 0.005;  // yaw: 위치보다 약간 노이즈

        P.setIdentity();
    }

    void init(double x, double y, double v, double theta) { // theta = yaw ori.z
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

    void update(double z_x, double z_y, double z_yaw) {
        // x, y, yaw 측정값
        Eigen::Vector3d z(z_x, z_y, z_yaw);
        Eigen::Matrix<double, 3, 4> H;
        H.setZero();
        H(0, 0) = 1.0;  // x 측정
        H(1, 1) = 1.0;  // y 측정
        H(2, 3) = 1.0;  // yaw 측정

        Eigen::Vector3d y = z - H * state;
        Eigen::Matrix3d S = H * P * H.transpose() + R3;
        Eigen::Matrix<double, 4, 3> K = P * H.transpose() * S.inverse();
        state = state + K * y;
        P = (Eigen::Matrix4d::Identity() - K * H) * P;
    }

    void update_with_velocity(double z_x, double z_y, double z_yaw, double z_v) {
        // x, y, yaw, v 모두 측정 (엔코더 사용 시)
        Eigen::Vector4d z(z_x, z_y, z_v, z_yaw);
        Eigen::Matrix4d H = Eigen::Matrix4d::Identity();  // 모든 상태 측정

        Eigen::Vector4d y = z - H * state;
        Eigen::Matrix4d S = H * P * H.transpose() + R4;
        Eigen::Matrix4d K = P * H.transpose() * S.inverse();
        state = state + K * y;
        P = (Eigen::Matrix4d::Identity() - K * H) * P;
    }

    bool is_initialized;
    Eigen::Vector4d state; // [x, y, v, theta]
    Eigen::Matrix4d P, Q;
    Eigen::Matrix3d R3;  // x, y, yaw 측정용
    Eigen::Matrix4d R4;  // x, y, v, yaw 측정용
};

#endif
