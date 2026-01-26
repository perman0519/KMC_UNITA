#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <vector>
#include <cmath>
#include <algorithm>

struct Path { std::vector<double> X, Y; };
struct Point { double x; double y; };

class PurePursuitController {
public:
    double compute_curvature(const Path& path, int my_idx, double cx, double cy, double cyaw, double ld) {
        int target_idx = -1;
        int path_size = static_cast<int>(path.X.size());
        double ld_sq = ld * ld;

        for (int i = my_idx; i < path_size; ++i) {
            double dx = path.X[i] - cx;
            double dy = path.Y[i] - cy;
            if (dx*dx + dy*dy >= ld_sq) {
                target_idx = i; break;
            }
        }
        if (target_idx == -1) target_idx = path_size - 1;

        double dx = path.X[target_idx] - cx;
        double dy = path.Y[target_idx] - cy;
        double local_y = -std::sin(cyaw) * dx + std::cos(cyaw) * dy;
        return (2.0 * local_y) / (ld_sq + 1e-9);
    }

    int find_closest_index(const Path& path, double x, double y) {
        int idx = 0;
        double min_sq_d = 1.0e18; // (1e9)^2

        int path_size = static_cast<int>(path.X.size());
        for (int i = 0; i < path_size; ++i) {
            double dx = path.X[i] - x;
            double dy = path.Y[i] - y;
            double sq_d = dx*dx + dy*dy;
            if (sq_d < min_sq_d) { min_sq_d = sq_d; idx = i; }
        }
        return idx;
    }
};

#endif
