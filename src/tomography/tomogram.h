#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <vector>

#include "tomography/kernels.h"

namespace pct {

struct TomogramConfig {
  // Map parameters
  float resolution = 0.3f;
  float slice_dh = 0.5f;

  // Traversability kernel parameters
  int kernel_size = 5;
  float interval_min = 0.5f;
  float interval_free = 0.65f;
  float slope_max = 0.40f;
  float step_max = 0.3f;
  float standable_ratio = 0.5f;
  float cost_barrier = 50.0f;

  // Inflation
  float safe_margin = 0.3f;
  float inflation = 0.2f;
};

// A Tomogram slices a 3D point cloud into horizontal layers and computes a
// traversability cost per cell. Mirrors pct_planner/tomography/scripts/tomogram.py
// but runs on the CPU using the kernels in kernels.h.
//
// After Run(points), the following fields are valid:
//   - layers_t: inflated traversability cost, shape [n_layers, map_dim_x, map_dim_y]
//   - trav_gx, trav_gy: gradient of traversability in x/y
//   - elev_g: ground elevation (NaN where no data)
//   - elev_c: ceiling elevation (NaN where no data)
//   - layer_count: number of layers after simplification
class Tomogram {
 public:
  explicit Tomogram(const TomogramConfig& cfg);

  // (Re)allocate buffers for the given map dimensions and slice range. Called
  // once per tomogram build.
  void InitMappingEnv(float center_x, float center_y, int map_dim_x,
                      int map_dim_y, int n_slice_init, float slice_h0);

  // Run the tomography + traversability + inflation + layer-simplification
  // pipeline on the given Nx3 point cloud. After this call, the layers_t
  // etc. accessors return the simplified stack.
  void Run(const float* points, std::size_t n_points);

  // Simplified outputs — row-major float32 grids sized
  // [layer_count * map_dim_x * map_dim_y].
  const std::vector<float>& layers_t() const { return layers_t_; }
  const std::vector<float>& trav_gx() const { return trav_gx_; }
  const std::vector<float>& trav_gy() const { return trav_gy_; }
  const std::vector<float>& elev_g_out() const { return elev_g_out_; }
  const std::vector<float>& elev_c_out() const { return elev_c_out_; }
  int layer_count() const { return layer_count_; }

  int map_dim_x() const { return map_dim_x_; }
  int map_dim_y() const { return map_dim_y_; }
  float resolution() const { return cfg_.resolution; }
  float slice_h0() const { return slice_h0_; }
  float slice_dh() const { return cfg_.slice_dh; }
  float center_x() const { return center_[0]; }
  float center_y() const { return center_[1]; }

 private:
  void ClearMap();
  void InitKernels();

  TomogramConfig cfg_;

  // Grid metadata
  float center_[2] = {0.0f, 0.0f};
  int map_dim_x_ = 0;
  int map_dim_y_ = 0;
  int n_slice_init_ = 0;
  float slice_h0_ = 0.0f;

  // Derived
  int half_trav_k_size_ = 0;
  int half_inf_k_size_ = 0;
  int standable_th_ = 0;
  float step_stand_ = 0.0f;

  // CPU kernels
  TomographyKernel tomography_kernel_{};
  TravKernel trav_kernel_{};
  InflationKernel inflation_kernel_{};
  std::vector<float> inf_table_;

  // Working buffers — flat [n_slice, x, y]
  std::vector<float> layers_g_;
  std::vector<float> layers_c_;
  std::vector<float> grad_mag_sq_;
  std::vector<float> grad_mag_max_;
  std::vector<float> trav_cost_;
  std::vector<float> inflated_cost_;

  // Simplified outputs
  std::vector<float> layers_t_;
  std::vector<float> trav_gx_;
  std::vector<float> trav_gy_;
  std::vector<float> elev_g_out_;
  std::vector<float> elev_c_out_;
  int layer_count_ = 0;
};

}  // namespace pct
