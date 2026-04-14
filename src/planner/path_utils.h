#pragma once

#include <Eigen/Core>
#include <cstddef>

namespace pct {

// CPU port of pct_planner/utils/path_utils.py.
// Given a planned Nx3 trajectory (in map coordinates) and the robot's current
// position, returns the lookahead waypoint along the path. Returns the last
// trajectory point if the lookahead distance exceeds the remaining path.
//
// Writes the waypoint into out[0..2]. Returns false if the path is empty.
bool GetWaypointFromTraj(const Eigen::MatrixXd& path, const Eigen::Vector3d& robot,
                        double lookahead_dist, double out[3]);

}  // namespace pct
