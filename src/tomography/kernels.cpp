#include "tomography/kernels.h"

#include <algorithm>
#include <cmath>

namespace pct {

namespace {
constexpr float kTrangeFreeGain = 20.0f;
constexpr float kGradStandGain = 15.0f;
constexpr float kGradCrossGain = 20.0f;
}  // namespace

void TomographyKernel::apply(const float* points, std::size_t n_points,
                             const float center[2], float* layers_g,
                             float* layers_c) const {
  const int layer_size = n_row * n_col;
  const int half_x = n_row / 2;
  const int half_y = n_col / 2;
  const float cx = center[0];
  const float cy = center[1];

  for (std::size_t i = 0; i < n_points; ++i) {
    const float px = points[i * 3 + 0];
    const float py = points[i * 3 + 1];
    const float pz = points[i * 3 + 2];

    const int idx_x = static_cast<int>(std::lround((px - cx) / resolution)) + half_x;
    const int idx_y = static_cast<int>(std::lround((py - cy) / resolution)) + half_y;
    if (idx_x < 0 || idx_x >= n_row || idx_y < 0 || idx_y >= n_col) continue;

    const int idx = n_col * idx_x + idx_y;
    for (int s_idx = 0; s_idx < n_slice; ++s_idx) {
      const float slice = slice_h0 + s_idx * slice_dh;
      const int block_idx = layer_size * s_idx + idx;
      if (pz <= slice) {
        if (pz > layers_g[block_idx]) layers_g[block_idx] = pz;
      } else {
        if (pz < layers_c[block_idx]) layers_c[block_idx] = pz;
      }
    }
  }
}

void TravKernel::apply(const float* interval, const float* grad_mag_sq,
                       const float* grad_mag_max, float* trav_cost,
                       int n_slice) const {
  const int layer_size = n_row * n_col;
  const int total = n_slice * layer_size;

  for (int i = 0; i < total; ++i) {
    const float iv = interval[i];
    if (iv < interval_min) {
      trav_cost[i] = cost_barrier;
      continue;
    }
    trav_cost[i] += std::max(0.0f, kTrangeFreeGain * (interval_free - iv));

    const float gsq = grad_mag_sq[i];
    if (gsq <= step_stand_sq) {
      trav_cost[i] += kGradStandGain * gsq / step_stand_sq;
      continue;
    }

    const float gmax = grad_mag_max[i];
    if (gmax > step_cross_sq) {
      trav_cost[i] = cost_barrier;
      continue;
    }

    const int idx_2d = i % layer_size;
    const int idx_x = idx_2d / n_col;
    const int idx_y = idx_2d % n_col;

    int standable_grids = 0;
    for (int dy = -half_kernel_size; dy <= half_kernel_size; ++dy) {
      for (int dx = -half_kernel_size; dx <= half_kernel_size; ++dx) {
        const int rx = idx_x + dx;
        const int ry = idx_y + dy;
        if (rx < 0 || rx >= n_row || ry < 0 || ry >= n_col) continue;
        const int rel = n_col * dx + dy + i;
        if (rel >= 0 && rel < total && grad_mag_sq[rel] < step_stand_sq) {
          ++standable_grids;
        }
      }
    }
    if (standable_grids < standable_th) {
      trav_cost[i] = cost_barrier;
    } else {
      trav_cost[i] += kGradCrossGain * gmax / step_cross_sq;
    }
  }
}

void InflationKernel::apply(const float* trav_cost, const float* score_table,
                            float* inflated_cost, int n_slice) const {
  const int layer_size = n_row * n_col;
  const int total = n_slice * layer_size;

  for (int i = 0; i < total; ++i) {
    const int idx_2d = i % layer_size;
    const int idx_x = idx_2d / n_col;
    const int idx_y = idx_2d % n_col;

    int counter = 0;
    for (int dy = -half_kernel_size; dy <= half_kernel_size; ++dy) {
      for (int dx = -half_kernel_size; dx <= half_kernel_size; ++dx) {
        const int rx = idx_x + dx;
        const int ry = idx_y + dy;
        if (rx >= 0 && rx < n_row && ry >= 0 && ry < n_col) {
          const int rel = n_col * dx + dy + i;
          if (rel >= 0 && rel < total) {
            const float v = trav_cost[rel] * score_table[counter];
            if (v > inflated_cost[i]) inflated_cost[i] = v;
          }
        }
        ++counter;
      }
    }
  }
}

}  // namespace pct
