#include "planner/path_utils.h"

#include <cmath>
#include <limits>

namespace pct {

namespace {
constexpr double kHeightPenalty = 3.0;
}  // namespace

bool GetWaypointFromTraj(const Eigen::MatrixXd& path, const Eigen::Vector3d& robot,
                         double lookahead_dist, double out[3]) {
  const int n = static_cast<int>(path.rows());
  if (n == 0) return false;

  // Closest point on path (weighted 3D distance, penalizing Z diffs to stay
  // on the correct floor).
  int closest_idx = 0;
  double best_d = std::numeric_limits<double>::infinity();
  for (int i = 0; i < n; ++i) {
    const double dx = path(i, 0) - robot.x();
    const double dy = path(i, 1) - robot.y();
    const double dz = (path(i, 2) - robot.z()) * kHeightPenalty;
    const double d = dx * dx + dy * dy + dz * dz;
    if (d < best_d) {
      best_d = d;
      closest_idx = i;
    }
  }

  double accumulated = 0.0;
  int wp_idx = n - 1;
  for (int i = closest_idx; i < n - 1; ++i) {
    const double dx = path(i + 1, 0) - path(i, 0);
    const double dy = path(i + 1, 1) - path(i, 1);
    const double seg = std::sqrt(dx * dx + dy * dy);
    if (accumulated + seg >= lookahead_dist) {
      wp_idx = i + 1;
      break;
    }
    accumulated += seg;
  }

  out[0] = path(wp_idx, 0);
  out[1] = path(wp_idx, 1);
  out[2] = path(wp_idx, 2);
  return true;
}

}  // namespace pct
