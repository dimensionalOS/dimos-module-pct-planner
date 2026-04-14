#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <vector>

namespace pct {

// CPU port of pct_planner/tomography/scripts/kernels.py.
// All grids are stored as flat float32 arrays of shape [n_slice, n_row, n_col]
// indexed as [s * (n_row * n_col) + x * n_col + y] (row-major x-major within
// each slice), matching the NumPy layout in the reference.

struct TomographyKernel {
  float resolution;
  int n_row;
  int n_col;
  int n_slice;
  float slice_h0;
  float slice_dh;

  // points: Nx3 flattened [px0,py0,pz0,px1,...], layers_g and layers_c are
  // pre-cleared to -inf/+inf respectively by Tomogram::clearMap.
  void apply(const float* points, std::size_t n_points, const float center[2],
             float* layers_g, float* layers_c) const;
};

struct TravKernel {
  int n_row;
  int n_col;
  int half_kernel_size;
  float interval_min;
  float interval_free;
  float step_cross_sq;
  float step_stand_sq;
  int standable_th;
  float cost_barrier;

  void apply(const float* interval, const float* grad_mag_sq,
             const float* grad_mag_max, float* trav_cost, int n_slice) const;
};

struct InflationKernel {
  int n_row;
  int n_col;
  int half_kernel_size;

  void apply(const float* trav_cost, const float* score_table,
             float* inflated_cost, int n_slice) const;
};

}  // namespace pct
