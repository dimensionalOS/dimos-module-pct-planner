#include "planner/pct_planner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace pct {

namespace {
constexpr int kMapDimPaddingCells = 4;  // ros-nav uses +4 cells, not a metric padding
constexpr double kGridStep = 0.2;  // step_cost_weight for OfflineElePlanner::InitMap
constexpr float kBigNegSentinel = -100.0f;
constexpr float kBigPosSentinel = 1e6f;

// GPMP trajectory column layout: [x, vx, ax, y, vy, ay] (6 columns).
// The reference Python does `np.concatenate([traj, layers], axis=-1)` then
// picks `y_idx = (cols - 1) // 2` against the 7-col matrix, which lands on
// col 3. Against the raw 6-col matrix that's `cols / 2`. Hard-code the
// constants so a future schema change doesn't silently pick an accel
// column (this was issue (d) in the run13 debug log).
constexpr int kGpmpColX = 0;
constexpr int kGpmpColY = 3;

// Waist-height offset (meters) added to the planned z so downstream
// visualizers show the path at the robot's body centerline instead of
// the ground plane. Matches the `+ 0.5` in the reference
// `transTrajGrid2Map` in planner_wrapper.py.
constexpr double kWaistHeightOffset = 0.5;
}  // namespace

TomogramPlanner::TomogramPlanner(const PlannerConfig& planner_cfg,
                                 const TomogramConfig& tomo_cfg)
    : planner_cfg_(planner_cfg), tomo_cfg_(tomo_cfg), tomogram_(tomo_cfg) {}

void TomogramPlanner::BuildTomogramFromCloud(const float* points,
                                             std::size_t n_points,
                                             float ground_h) {
  if (n_points == 0) return;

  // NaN filter + bounding-box pass in one go.  ros-nav's tomogram.py
  // does `points = points[~cp.isnan(points).any(axis=1)]` before the
  // kernel runs — we replicate that here by building a filtered copy
  // and passing it to `Tomogram::Run` below, so the kernel never sees
  // NaN/Inf coordinates.
  std::vector<float> filtered;
  filtered.reserve(n_points * 3);
  float min_x = std::numeric_limits<float>::infinity();
  float max_x = -std::numeric_limits<float>::infinity();
  float min_y = std::numeric_limits<float>::infinity();
  float max_y = -std::numeric_limits<float>::infinity();
  float min_z = std::numeric_limits<float>::infinity();
  float max_z = -std::numeric_limits<float>::infinity();
  for (std::size_t i = 0; i < n_points; ++i) {
    const float px = points[i * 3 + 0];
    const float py = points[i * 3 + 1];
    const float pz = points[i * 3 + 2];
    if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz)) continue;
    filtered.push_back(px);
    filtered.push_back(py);
    filtered.push_back(pz);
    if (px < min_x) min_x = px;
    if (px > max_x) max_x = px;
    if (py < min_y) min_y = py;
    if (py > max_y) max_y = py;
    if (pz < min_z) min_z = pz;
    if (pz > max_z) max_z = pz;
  }
  if (filtered.empty() || !std::isfinite(min_x)) return;
  const std::size_t n_filtered = filtered.size() / 3;

  // Match ros-nav `pct_planner.py::buildTomogramFromCloud` (lines 85-88):
  //     map_dim_{x,y} = ceil(span / resolution) + 4
  //     n_slice_init  = ceil((max_z - min_z) / slice_dh)
  //     slice_h0      = ground_h + slice_dh
  //
  // Previously this function used +2m of metric padding, a fudge-
  // based n_slice, and `slice_h0 = floor(min_z) - 1.0f`.  The old
  // slice_h0 left layer 0 below every real point, which turned the
  // ground layer into an all-NaN sheet downstream (the `be2370e force
  // waypoint/path z to robot z` commit was patching the symptom at
  // the output stage).
  const float cx = 0.5f * (min_x + max_x);
  const float cy = 0.5f * (min_y + max_y);
  const float span_x = max_x - min_x;
  const float span_y = max_y - min_y;
  const int nx = std::max(
      16, static_cast<int>(std::ceil(span_x / tomo_cfg_.resolution)) + kMapDimPaddingCells);
  const int ny = std::max(
      16, static_cast<int>(std::ceil(span_y / tomo_cfg_.resolution)) + kMapDimPaddingCells);

  const float slice_h0 = ground_h + tomo_cfg_.slice_dh;
  // Slice span runs from `ground_h` (or the observed min_z, whichever
  // is lower — we never want the cloud to extend below slice 0) up to
  // `max_z`.  Divide by slice_dh to get the initial slice count.
  const float slice_span_low = std::min(ground_h, min_z);
  const int n_slice = std::max(
      2, static_cast<int>(std::ceil((max_z - slice_span_low) / tomo_cfg_.slice_dh)));

  tomogram_.InitMappingEnv(cx, cy, nx, ny, n_slice, slice_h0);
  tomogram_.Run(filtered.data(), n_filtered);

  center_ = Eigen::Vector2d(cx, cy);
  map_dim_ = {nx, ny};
  offset_ = {nx / 2, ny / 2};
  resolution_ = tomo_cfg_.resolution;
  n_slice_ = tomogram_.layer_count();
  slice_h0_ = slice_h0;
  slice_dh_ = tomo_cfg_.slice_dh;

  InitEleplannerFromTomogram();
  tomogram_loaded_ = true;
}

void TomogramPlanner::InitEleplannerFromTomogram() {
  const int nx = map_dim_[0];
  const int ny = map_dim_[1];
  const int n_slice = n_slice_;
  const int layer_size = nx * ny;
  const int rows = n_slice * nx;

  // OfflineElePlanner::InitMap expects [rows, cols] where rows = n_slice * nx,
  // cols = ny. Same layout as np.reshape(-1, shape[-1]) in planner_wrapper.py.
  Eigen::MatrixXd cost_map(rows, ny);
  Eigen::MatrixXd elev_g(rows, ny);
  Eigen::MatrixXd elev_c(rows, ny);
  Eigen::MatrixXd grad_x(rows, ny);
  Eigen::MatrixXd grad_y(rows, ny);
  Eigen::MatrixXd gateway(rows, ny);

  const auto& tsrc = tomogram_.layers_t();
  const auto& gsrc = tomogram_.elev_g_out();
  const auto& csrc = tomogram_.elev_c_out();
  const auto& gxsrc = tomogram_.trav_gx();
  const auto& gysrc = tomogram_.trav_gy();

  for (int s = 0; s < n_slice; ++s) {
    for (int x = 0; x < nx; ++x) {
      for (int y = 0; y < ny; ++y) {
        const int src_idx = s * layer_size + x * ny + y;
        const int dst_row = s * nx + x;
        cost_map(dst_row, y) = static_cast<double>(tsrc[src_idx]);
        const float g = gsrc[src_idx];
        elev_g(dst_row, y) = std::isnan(g) ? kBigNegSentinel : static_cast<double>(g);
        const float c = csrc[src_idx];
        elev_c(dst_row, y) = std::isnan(c) ? kBigPosSentinel : static_cast<double>(c);
        // planner_wrapper.py maps numpy trav_gy → grad_x (InitMap arg)
        // and -trav_gx → grad_y. Preserve that swap.
        grad_x(dst_row, y) = static_cast<double>(gysrc[src_idx]);
        grad_y(dst_row, y) = -static_cast<double>(gxsrc[src_idx]);
        gateway(dst_row, y) = 0.0;
      }
    }
  }

  // Match planner_wrapper.py: gateway detection from trav diffs + elev_g diffs.
  // Thresholds hard-coded to reference values.
  for (int s = 0; s + 1 < n_slice; ++s) {
    for (int x = 0; x < nx; ++x) {
      for (int y = 0; y < ny; ++y) {
        const int lo = s * layer_size + x * ny + y;
        const int hi = (s + 1) * layer_size + x * ny + y;
        const float diff_t = tsrc[hi] - tsrc[lo];
        const float diff_g = std::fabs(gsrc[hi] - gsrc[lo]);
        const bool mask_g_ok = diff_g < 0.1f && !std::isnan(gsrc[hi]);
        const bool up = (diff_t < -8.0f) && mask_g_ok;
        const bool dn = (diff_t > 8.0f) && (diff_g < 0.1f) && !std::isnan(gsrc[lo]);
        if (up) gateway(s * nx + x, y) = 2.0;
        if (dn) gateway((s + 1) * nx + x, y) = -2.0;
      }
    }
  }

  ele_planner_ = std::make_unique<OfflineElePlanner>(planner_cfg_.max_heading_rate,
                                                     planner_cfg_.use_quintic);
  ele_planner_->InitMap(planner_cfg_.astar_cost_threshold,
                        planner_cfg_.safe_cost_margin, resolution_, n_slice,
                        kGridStep, cost_map, elev_g, elev_c, gateway, grad_x,
                        grad_y);
  ele_planner_->set_sample_interval(planner_cfg_.sample_interval);
  ele_planner_->set_interpolate_num(planner_cfg_.interpolate_num);
  ele_planner_->set_max_iterations(planner_cfg_.max_iterations);
  ele_planner_->set_lambda_initial(planner_cfg_.lambda_initial);
  ele_planner_->set_qc_position(planner_cfg_.qc_position);
  ele_planner_->set_qc_heading(planner_cfg_.qc_heading);
}

Eigen::Vector2i TomogramPlanner::Pos2Idx(const Eigen::Vector2d& pos) const {
  const Eigen::Vector2d rel = pos - center_;
  int ix = static_cast<int>(std::lround(rel.x() / resolution_)) + offset_[0];
  int iy = static_cast<int>(std::lround(rel.y() / resolution_)) + offset_[1];
  // Clamp to grid. Out-of-grid start/goal would otherwise feed negative or
  // >= map_dim indices to the A* search, producing undefined behavior
  // inside OfflineElePlanner::Plan.
  if (ix < 0) ix = 0;
  if (iy < 0) iy = 0;
  if (ix >= map_dim_[0]) ix = map_dim_[0] - 1;
  if (iy >= map_dim_[1]) iy = map_dim_[1] - 1;
  // Axis swap: the tomogram stores [slice, x_map, y_map] but the A* /
  // ele_planner index convention is [slice, row, col] where row = y_map
  // and col = x_map. Reference `planner_wrapper.py::Pos2Idx` returns
  // `(iy, ix)` — this was the source of the transTrajGrid2Map column
  // confusion during bring-up. Treat `ix` / `iy` here as map-frame
  // coordinates, and the `(iy, ix)` swap as the bridge to planner frame.
  return {iy, ix};
}

Eigen::MatrixXd TomogramPlanner::Plan(const Eigen::Vector3d& start,
                                      const Eigen::Vector3d& goal) {
  if (!tomogram_loaded_ || !ele_planner_) return Eigen::MatrixXd();

  Eigen::Vector3i start_idx;
  Eigen::Vector3i goal_idx;

  // Layer simplification collapses the original tomogram slices down to a
  // minimal set (often 1 or 2 for single-floor scenes), so a naive
  // (z - slice_h0) / slice_dh lookup against the *simplified* layer count is
  // meaningless. Default both ends to the ground layer (layer 0); for
  // multi-floor the A* traversability+gateway logic will transition up.
  start_idx[0] = 0;
  goal_idx[0] = 0;
  (void)start.z();
  (void)goal.z();

  const Eigen::Vector2i s2 = Pos2Idx(start.head<2>());
  const Eigen::Vector2i g2 = Pos2Idx(goal.head<2>());
  start_idx[1] = s2.x();
  start_idx[2] = s2.y();
  goal_idx[1] = g2.x();
  goal_idx[2] = g2.y();

  if (!ele_planner_->Plan(start_idx, goal_idx, true)) {
    return Eigen::MatrixXd();
  }

  // `use_quintic=true` runs the GPMPOptimizer (wnoj) inside the ele_planner,
  // so fetch the result matrix from that optimizer. The wnoa branch only has
  // a populated trajectory_ when use_quintic=false.
  Eigen::MatrixXd traj;
  Eigen::VectorXd layers;
  Eigen::VectorXd heights;
  if (planner_cfg_.use_quintic) {
    const auto& optimizer = ele_planner_->get_trajectory_optimizer_wnoj();
    traj = optimizer.GetResultMatrix();
    layers = optimizer.GetResultLayers();
    heights = optimizer.GetResultHeight();
  } else {
    const auto& optimizer = ele_planner_->get_trajectory_optimizer();
    traj = optimizer.GetResultMatrix();
    layers = optimizer.GetResultLayers();
    heights = optimizer.GetResultHeight();
  }
  if (traj.rows() == 0) return Eigen::MatrixXd();
  if (traj.cols() <= kGpmpColY) {
    // Schema changed — don't silently produce garbage trajectories.
    return Eigen::MatrixXd();
  }

  const int n = static_cast<int>(traj.rows());
  Eigen::MatrixXd traj_3d(n, 3);

  // transTrajGrid2Map port: grid indices → map-frame metric coordinates,
  // undoing the Pos2Idx offset/axis-swap.
  const double offx = static_cast<double>(map_dim_[1] / 2);
  const double offy = static_cast<double>(map_dim_[0] / 2);
  for (int i = 0; i < n; ++i) {
    const double gx = traj(i, kGpmpColX) - offx;
    const double gy = traj(i, kGpmpColY) - offy;
    const double map_x = gy * resolution_ + center_.x();
    const double map_y = gx * resolution_ + center_.y();
    const double map_z = static_cast<double>(heights(i)) + kWaistHeightOffset;
    traj_3d(i, 0) = map_x;
    traj_3d(i, 1) = map_y;
    traj_3d(i, 2) = map_z;
  }
  return traj_3d;
}

}  // namespace pct
