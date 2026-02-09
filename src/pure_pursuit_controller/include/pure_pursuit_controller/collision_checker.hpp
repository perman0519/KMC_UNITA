#ifndef COLLISION_CHECKER_HPP
#define COLLISION_CHECKER_HPP

#include "controller.hpp"
#include <map>

struct ConflictRule {
  int other_id;
  const Path *other_path_ptr;
};
struct SegmentRule {
  const Path *my_segment_ptr;
  std::vector<ConflictRule> conflicts;
};

class CollisionChecker {
public:
  bool is_on_path(const Path &path, double x, double y,
                  double threshold = 0.2) {
    int idx = find_closest_index_internal(path, x, y);
    return std::hypot(path.X[idx] - x, path.Y[idx] - y) < threshold;
  }

  bool check_hv_collision(const Path &my_path, int my_idx,
                          const std::map<int, Point> &others_pos,
                          const std::map<int, Path> &others_paths,
                          const std::map<int, bool> &others_received) {
    int my_path_size = static_cast<int>(my_path.X.size());
    double dist_th = 0.65;
    double dist_sq_threshold = dist_th * dist_th;

    for (int id : {19, 20}) {
      if (others_received.find(id) == others_received.end() ||
          !others_received.at(id))
        continue;
      const auto &hv_path = others_paths.at(id);
      int hv_path_size = static_cast<int>(hv_path.X.size());
      int hv_idx = find_closest_index_internal(hv_path, others_pos.at(id).x,
                                               others_pos.at(id).y);

      double my_accum = 0.0;
      for (int i = my_idx; i < my_path_size - 1 && my_accum < 1.5; ++i) {
        double dx_my = my_path.X[i + 1] - my_path.X[i];
        double dy_my = my_path.Y[i + 1] - my_path.Y[i];
        my_accum += std::sqrt(dx_my * dx_my + dy_my * dy_my);

        double hv_accum = 0.0;
        for (int j = hv_idx; j < hv_path_size - 1 && hv_accum < 1.5; ++j) {
          double dx_hv = hv_path.X[j + 1] - hv_path.X[j];
          double dy_hv = hv_path.Y[j + 1] - hv_path.Y[j];
          hv_accum += std::sqrt(dx_hv * dx_hv + dy_hv * dy_hv);

          double dx_cross = my_path.X[i] - hv_path.X[j];
          double dy_cross = my_path.Y[i] - hv_path.Y[j];

          if (dx_cross * dx_cross + dy_cross * dy_cross < dist_sq_threshold) {
            if (hv_accum < my_accum)
              return true;
            // return true;
          }
        }
      }
    }
    return false;
  }

  bool check_hardcoded(const Path &my_path, int my_idx, int other_id,
                       const Path &other_path,
                       const std::map<int, Point> &others_pos,
                       const std::map<int, bool> &others_received,
                       int my_logical_id) {
    if (others_received.find(other_id) == others_received.end() ||
        !others_received.at(other_id))
      return false;

    if (!is_on_path(other_path, others_pos.at(other_id).x,
                    others_pos.at(other_id).y, 0.2)) {
      return false;
    }

    int other_idx = find_closest_index_internal(
        other_path, others_pos.at(other_id).x, others_pos.at(other_id).y);
    int my_path_size = static_cast<int>(my_path.X.size());
    int other_path_size = static_cast<int>(other_path.X.size());

    double my_accum = 0.0;
    double dist_th = 0.25;
    double dist_sq_threshold = dist_th * dist_th; // 의미:

    for (int i = my_idx; i < my_path_size - 1 && my_accum < 1.3; ++i) {
      double dx_my = my_path.X[i + 1] - my_path.X[i];
      double dy_my = my_path.Y[i + 1] - my_path.Y[i];
      my_accum += std::sqrt(dx_my * dx_my + dy_my * dy_my);

      double other_accum = 0.0;
      for (int j = other_idx; j < other_path_size - 1 && other_accum < 1.5;
           ++j) {
        double dx_other = other_path.X[j + 1] - other_path.X[j];
        double dy_other = other_path.Y[j + 1] - other_path.Y[j];
        other_accum += std::sqrt(dx_other * dx_other + dy_other * dy_other);

        double dx_cross = my_path.X[i] - other_path.X[j];
        double dy_cross = my_path.Y[i] - other_path.Y[j];

        if (dx_cross * dx_cross + dy_cross * dy_cross < dist_sq_threshold) {
          if (my_accum > other_accum + 0.15)
            return true;
          if (std::abs(my_accum - other_accum) <= 0.15 &&
              my_logical_id > other_id)
            return true;
        }
      }
    }
    return false;
  }

private:
  int find_closest_index_internal(const Path &path, double x, double y) {
    int idx = 0;
    double min_sq_d = 1.0e18; // Use a large number, (1e9)^2

    int path_size = static_cast<int>(path.X.size());
    for (int i = 0; i < path_size; ++i) {
      double dx = path.X[i] - x;
      double dy = path.Y[i] - y;
      double sq_d = dx * dx + dy * dy;
      if (sq_d < min_sq_d) {
        min_sq_d = sq_d;
        idx = i;
      }
    }
    return idx;
  }
};

#endif
