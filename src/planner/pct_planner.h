#pragma once

#include <Eigen/Core>
#include <array>
#include <memory>
#include <vector>

#include "ele_planner/offline_ele_planner.h"
#include "tomography/tomogram.h"

namespace pct {

struct PlannerConfig {
  // Match pct_planner reference param.py defaults.
  double astar_cost_threshold = 50.0;
  double safe_cost_margin = 15.0;
  double max_heading_rate = 10.0;
  bool use_quintic = true;
  int sample_interval = 5;
  int interpolate_num = 5;
  int max_iterations = 100;
  double lambda_initial = 10.0;
  double qc_position = 0.05;
  double qc_heading = 0.005;
};

// Orchestrator: loads a Tomogram, feeds it to OfflineElePlanner, and extracts
// a 3D trajectory in map coordinates.
class TomogramPlanner {
 public:
  TomogramPlanner(const PlannerConfig& planner_cfg, const TomogramConfig& tomo_cfg);

  // Build a tomogram from a point cloud and install it in the underlying
  // ele_planner. The grid is centered on the point cloud's bounding box
  // center, sized to span the extents + 2m padding.
  void BuildTomogramFromCloud(const float* points, std::size_t n_points);

  // Plan a 3D path from start to goal (map coordinates, meters). Returns an
  // Nx3 matrix of [x, y, z] in map frame, or an empty matrix if planning
  // failed.
  Eigen::MatrixXd Plan(const Eigen::Vector3d& start, const Eigen::Vector3d& goal);

  bool has_tomogram() const { return tomogram_loaded_; }
  const Tomogram& tomogram() const { return tomogram_; }

 private:
  Eigen::Vector2i Pos2Idx(const Eigen::Vector2d& pos) const;
  void InitEleplannerFromTomogram();

  PlannerConfig planner_cfg_;
  TomogramConfig tomo_cfg_;

  Tomogram tomogram_;
  std::unique_ptr<OfflineElePlanner> ele_planner_;

  bool tomogram_loaded_ = false;
  std::array<int, 2> map_dim_ = {0, 0};
  std::array<int, 2> offset_ = {0, 0};
  Eigen::Vector2d center_ = Eigen::Vector2d::Zero();
  double resolution_ = 0.0;
  int n_slice_ = 0;
  double slice_h0_ = 0.0;
  double slice_dh_ = 0.0;
};

}  // namespace pct
