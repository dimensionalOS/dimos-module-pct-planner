#include "planner/path_utils.h"

#include <cmath>
#include <limits>

namespace pct {

namespace {
// Weight on z-distance when selecting the closest path point to the robot.
// Keeps lookahead on the current floor: a point two layers above but
// directly overhead has d_xy=0 but `kLayerSeparationWeight * d_z`
// penalty that pushes it out of the "closest" slot in favor of a
// same-floor neighbor a few cells ahead. 3.0 is the upstream value.
constexpr double kLayerSeparationWeight = 3.0;
}  // namespace

bool GetWaypointFromTraj(const Eigen::MatrixXd& path, const Eigen::Vector3d& robot,
                         double lookahead_dist, double out[3]) {
  const int n = static_cast<int>(path.rows());
  if (n == 0) return false;

  // Closest point on path. Use the same kLayerSeparationWeight metric for
  // both the closest-point search and the lookahead accumulation so they
  // agree on "which floor we are on" for vertical paths.
  int closest_idx = 0;
  double best_d = std::numeric_limits<double>::infinity();
  for (int i = 0; i < n; ++i) {
    const double dx = path(i, 0) - robot.x();
    const double dy = path(i, 1) - robot.y();
    const double dz = (path(i, 2) - robot.z()) * kLayerSeparationWeight;
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
    const double dz = (path(i + 1, 2) - path(i, 2)) * kLayerSeparationWeight;
    const double seg = std::sqrt(dx * dx + dy * dy + dz * dz);
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
