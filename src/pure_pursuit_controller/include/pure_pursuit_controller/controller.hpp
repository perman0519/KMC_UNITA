#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <vector>
#include <cmath>
#include <algorithm>

struct Path { std::vector<double> X, Y; };
struct Point { double x; double y; };

/**
 * @class PurePursuitController
 * @brief 횡방향 제어를 위한 Pure Pursuit 및 보조 함수 제공
 *
 * 이 클래스는 3가지 횡방향 제어 모드를 지원합니다:
 * - Mode 0: Pure Pursuit (부드러움 우선)
 * - Mode 1: Path PID (정확도 우선)
 * - Mode 2: Hybrid (균형잡힌 제어, 코너 컷팅 방지)
 */
class PurePursuitController {
public:
    /**
     * @brief Pure Pursuit 알고리즘으로 곡률 계산
     *
     * Lookahead distance만큼 앞의 점을 찾아 그 점으로 향하는 곡률을 계산합니다.
     *
     * @param path 경로 데이터 (X, Y 좌표 벡터)
     * @param my_idx 현재 경로상의 인덱스
     * @param cx 현재 차량 x 좌표
     * @param cy 현재 차량 y 좌표
     * @param cyaw 현재 차량 방향각 [rad]
     * @param ld Lookahead distance [m]
     * @return 곡률 kappa [1/m]
     *
     * @note omega = v * kappa 관계식으로 각속도를 구합니다.
     * @note ld가 클수록 부드럽지만 코너 컷팅 발생 가능
     */
    double compute_curvature(const Path& path, int my_idx, double cx, double cy, double cyaw, double ld) {
        int target_idx = -1;
        int path_size = static_cast<int>(path.X.size());
        double ld_sq = ld * ld;

        // Lookahead distance 이상 떨어진 첫 번째 점 찾기
        for (int i = my_idx; i < path_size; ++i) {
            double dx = path.X[i] - cx;
            double dy = path.Y[i] - cy;
            if (dx*dx + dy*dy >= ld_sq) {
                target_idx = i;
                break;
            }
        }
        if (target_idx == -1) target_idx = path_size - 1;

        // 차량 좌표계로 변환하여 lateral offset 계산
        double dx = path.X[target_idx] - cx;
        double dy = path.Y[target_idx] - cy;
        double local_y = -std::sin(cyaw) * dx + std::cos(cyaw) * dy;

        // Pure Pursuit 곡률 공식: k = 2*y / L^2
        return (2.0 * local_y) / (ld_sq + 1e-9);
    }

    /**
     * @brief 경로에서 가장 가까운 점의 인덱스 찾기
     *
     * @param path 경로 데이터
     * @param x 현재 x 좌표
     * @param y 현재 y 좌표
     * @return 가장 가까운 경로 상의 인덱스
     */
    int find_closest_index(const Path& path, double x, double y) {
        int idx = 0;
        double min_sq_d = 1.0e18; // (1e9)^2

        int path_size = static_cast<int>(path.X.size());
        for (int i = 0; i < path_size; ++i) {
            double dx = path.X[i] - x;
            double dy = path.Y[i] - y;
            double sq_d = dx*dx + dy*dy;
            if (sq_d < min_sq_d) {
                min_sq_d = sq_d;
                idx = i;
            }
        }
        return idx;
    }

    /**
     * @brief 경로의 방향각 계산 (Path PID용)
     *
     * 현재 idx에서 약간 앞(idx+3)까지의 방향을 계산하여
     * 차량이 따라가야 할 목표 방향을 제공합니다.
     *
     * @param path 경로 데이터
     * @param idx 현재 경로 인덱스
     * @return 경로 방향각 [rad]
     *
     * @note idx+3을 사용하여 약간의 여유를 둠 (노이즈 감소)
     * @note Mode 1, 2에서 yaw error 계산에 사용
     */
    double compute_path_yaw(const Path& path, int idx) {
        int path_size = static_cast<int>(path.X.size());
        if (idx >= path_size - 1) return 0.0;

        // 현재부터 3칸 앞까지의 방향 (노이즈 감소)
        double dx = path.X[idx + 3] - path.X[idx];
        double dy = path.Y[idx + 3] - path.Y[idx];
        return std::atan2(dy, dx);
    }

    /**
     * @brief 현재 위치에서 경로까지의 횡방향 오차 계산
     *
     * 차량 좌표계에서 경로까지의 수직 거리를 계산합니다.
     * 양수: 경로가 차량 왼쪽, 음수: 경로가 차량 오른쪽
     *
     * @param path 경로 데이터
     * @param idx 현재 경로 인덱스
     * @param cx 차량 x 좌표
     * @param cy 차량 y 좌표
     * @param cyaw 차량 방향각 [rad]
     * @return 횡방향 오차 [m]
     *
     * @note 현재 미사용 (향후 Mode 3 추가 시 활용 가능)
     */
    double compute_lateral_error(const Path& path, int idx, double cx, double cy, double cyaw) {
        if (idx >= static_cast<int>(path.X.size()) - 1) return 0.0;

        double dx = path.X[idx] - cx;
        double dy = path.Y[idx] - cy;

        // 차량 좌표계에서의 횡방향 오차
        double lateral_error = -std::sin(cyaw) * dx + std::cos(cyaw) * dy;
        return lateral_error;
    }
};

#endif
