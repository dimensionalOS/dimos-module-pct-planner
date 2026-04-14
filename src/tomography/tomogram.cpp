#include "tomography/tomogram.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace pct {

namespace {
constexpr float kBigNeg = -1e6f;
constexpr float kBigPos = 1e6f;
constexpr float kTomoStepScale = 1.2f;
}  // namespace

Tomogram::Tomogram(const TomogramConfig& cfg) : cfg_(cfg) {
  half_trav_k_size_ = cfg_.kernel_size / 2;
  const int k = 2 * half_trav_k_size_ + 1;
  standable_th_ = static_cast<int>(cfg_.standable_ratio * k * k) - 1;
  step_stand_ = kTomoStepScale * cfg_.resolution * std::tan(cfg_.slope_max);
  half_inf_k_size_ =
      static_cast<int>((cfg_.safe_margin + cfg_.inflation) / cfg_.resolution);
}

void Tomogram::InitMappingEnv(float center_x, float center_y, int map_dim_x,
                              int map_dim_y, int n_slice_init, float slice_h0) {
  center_[0] = center_x;
  center_[1] = center_y;
  map_dim_x_ = map_dim_x;
  map_dim_y_ = map_dim_y;
  n_slice_init_ = n_slice_init;
  slice_h0_ = slice_h0;

  const std::size_t voxels =
      static_cast<std::size_t>(n_slice_init_) * map_dim_x_ * map_dim_y_;
  layers_g_.assign(voxels, 0.0f);
  layers_c_.assign(voxels, 0.0f);
  grad_mag_sq_.assign(voxels, 0.0f);
  grad_mag_max_.assign(voxels, 0.0f);
  trav_cost_.assign(voxels, 0.0f);
  inflated_cost_.assign(voxels, 0.0f);

  InitKernels();
}

void Tomogram::InitKernels() {
  tomography_kernel_ = TomographyKernel{
      cfg_.resolution, map_dim_x_,       map_dim_y_,
      n_slice_init_,   slice_h0_,        cfg_.slice_dh};

  trav_kernel_ = TravKernel{
      map_dim_x_,
      map_dim_y_,
      half_trav_k_size_,
      cfg_.interval_min,
      cfg_.interval_free,
      cfg_.step_max * cfg_.step_max,
      step_stand_ * step_stand_,
      standable_th_,
      cfg_.cost_barrier};

  inflation_kernel_ =
      InflationKernel{map_dim_x_, map_dim_y_, half_inf_k_size_};

  // Inflation distance table (radial falloff, saturated to [0, 1]).
  const int inf_k = 2 * half_inf_k_size_ + 1;
  inf_table_.assign(static_cast<std::size_t>(inf_k) * inf_k, 0.0f);
  for (int i = 0; i < inf_k; ++i) {
    for (int j = 0; j < inf_k; ++j) {
      const float dx = cfg_.resolution * (i - half_inf_k_size_);
      const float dy = cfg_.resolution * (j - half_inf_k_size_);
      const float dist = std::sqrt(dx * dx + dy * dy);
      float v = 1.0f - (dist - cfg_.inflation) / (cfg_.safe_margin + cfg_.resolution);
      if (v < 0.0f) v = 0.0f;
      if (v > 1.0f) v = 1.0f;
      inf_table_[static_cast<std::size_t>(i) * inf_k + j] = v;
    }
  }
}

void Tomogram::ClearMap() {
  std::fill(layers_g_.begin(), layers_g_.end(), kBigNeg);
  std::fill(layers_c_.begin(), layers_c_.end(), kBigPos);
  std::fill(grad_mag_sq_.begin(), grad_mag_sq_.end(), 0.0f);
  std::fill(grad_mag_max_.begin(), grad_mag_max_.end(), 0.0f);
  std::fill(trav_cost_.begin(), trav_cost_.end(), 0.0f);
  std::fill(inflated_cost_.begin(), inflated_cost_.end(), 0.0f);
}

void Tomogram::Run(const float* points, std::size_t n_points) {
  ClearMap();
  tomography_kernel_.apply(points, n_points, center_, layers_g_.data(),
                           layers_c_.data());

  const int nx = map_dim_x_;
  const int ny = map_dim_y_;
  const int layer_size = nx * ny;

  // Gradient magnitudes — match the numpy slicing layout in tomogram.py.
  // grad_mag_sq[s, x, y] = max((g[s,x,y]-g[s,x-1,y])^2, (g[s,x,y]-g[s,x+1,y])^2)
  //                     + max((g[s,x,y]-g[s,x,y-1])^2, (g[s,x,y]-g[s,x,y+1])^2)
  // Only written for interior cells (1 <= x <= nx-2, 1 <= y <= ny-2).
  for (int s = 0; s < n_slice_init_; ++s) {
    const float* g = layers_g_.data() + static_cast<std::size_t>(s) * layer_size;
    float* gsq =
        grad_mag_sq_.data() + static_cast<std::size_t>(s) * layer_size;
    float* gmx =
        grad_mag_max_.data() + static_cast<std::size_t>(s) * layer_size;
    for (int x = 1; x < nx - 1; ++x) {
      for (int y = 1; y < ny - 1; ++y) {
        const int idx = x * ny + y;
        const float c = g[idx];
        const float dxp = c - g[(x - 1) * ny + y];
        const float dxn = c - g[(x + 1) * ny + y];
        const float dyp = c - g[x * ny + (y - 1)];
        const float dyn = c - g[x * ny + (y + 1)];
        const float dx_sq = std::max(dxp * dxp, dxn * dxn);
        const float dy_sq = std::max(dyp * dyp, dyn * dyn);
        gsq[idx] = dx_sq + dy_sq;
        gmx[idx] = std::max(dx_sq, dy_sq);
      }
    }
  }

  // Interval = ceiling - ground.
  std::vector<float> interval(layers_c_.size());
  for (std::size_t i = 0; i < interval.size(); ++i) {
    interval[i] = layers_c_[i] - layers_g_[i];
  }

  trav_kernel_.apply(interval.data(), grad_mag_sq_.data(),
                     grad_mag_max_.data(), trav_cost_.data(), n_slice_init_);

  inflation_kernel_.apply(trav_cost_.data(), inf_table_.data(),
                          inflated_cost_.data(), n_slice_init_);

  // Layer simplification — keep slices that introduce new reachable regions.
  std::vector<int> idx_simp;
  idx_simp.push_back(0);
  if (n_slice_init_ > 1) {
    int l_idx = 0;
    int m_idx = 1;
    while (m_idx < n_slice_init_ - 2) {
      bool unique = false;
      const float* g_l = layers_g_.data() + static_cast<std::size_t>(l_idx) * layer_size;
      const float* g_m = layers_g_.data() + static_cast<std::size_t>(m_idx) * layer_size;
      const float* g_u = layers_g_.data() + static_cast<std::size_t>(m_idx + 1) * layer_size;
      const float* t_l = inflated_cost_.data() +
                         static_cast<std::size_t>(l_idx) * layer_size;
      const float* t_m = inflated_cost_.data() +
                         static_cast<std::size_t>(m_idx) * layer_size;
      for (int i = 0; i < layer_size; ++i) {
        const bool mask_l_g = (g_m[i] - g_l[i]) > 0.0f;
        const bool mask_l_t = t_l[i] > t_m[i];
        const bool mask_u_g = (g_u[i] - g_m[i]) > 0.0f;
        const bool mask_t = t_m[i] < cfg_.cost_barrier;
        if ((mask_l_g || mask_l_t) && mask_u_g && mask_t) {
          unique = true;
          break;
        }
      }
      if (unique) {
        idx_simp.push_back(m_idx);
        l_idx = m_idx;
      }
      ++m_idx;
    }
    idx_simp.push_back(m_idx);
  }

  layer_count_ = static_cast<int>(idx_simp.size());
  const std::size_t out_voxels = static_cast<std::size_t>(layer_count_) * layer_size;
  layers_t_.assign(out_voxels, 0.0f);
  trav_gx_.assign(out_voxels, 0.0f);
  trav_gy_.assign(out_voxels, 0.0f);
  elev_g_out_.assign(out_voxels, 0.0f);
  elev_c_out_.assign(out_voxels, 0.0f);

  const float nan_val = std::numeric_limits<float>::quiet_NaN();
  for (int k = 0; k < layer_count_; ++k) {
    const int s = idx_simp[k];
    const float* tsrc =
        inflated_cost_.data() + static_cast<std::size_t>(s) * layer_size;
    const float* gsrc =
        layers_g_.data() + static_cast<std::size_t>(s) * layer_size;
    const float* csrc =
        layers_c_.data() + static_cast<std::size_t>(s) * layer_size;
    float* tdst = layers_t_.data() + static_cast<std::size_t>(k) * layer_size;
    float* gdst = elev_g_out_.data() + static_cast<std::size_t>(k) * layer_size;
    float* cdst = elev_c_out_.data() + static_cast<std::size_t>(k) * layer_size;
    for (int i = 0; i < layer_size; ++i) {
      tdst[i] = tsrc[i];
      gdst[i] = gsrc[i] > kBigNeg ? gsrc[i] : nan_val;
      cdst[i] = csrc[i] < kBigPos ? csrc[i] : nan_val;
    }

    // trav_gx / trav_gy — central differences on the simplified slice.
    float* gxdst = trav_gx_.data() + static_cast<std::size_t>(k) * layer_size;
    float* gydst = trav_gy_.data() + static_cast<std::size_t>(k) * layer_size;
    for (int x = 1; x < nx - 1; ++x) {
      for (int y = 0; y < ny; ++y) {
        gxdst[x * ny + y] = tsrc[(x + 1) * ny + y] - tsrc[(x - 1) * ny + y];
      }
    }
    for (int x = 0; x < nx; ++x) {
      for (int y = 1; y < ny - 1; ++y) {
        gydst[x * ny + y] = tsrc[x * ny + (y + 1)] - tsrc[x * ny + (y - 1)];
      }
    }
  }
}

}  // namespace pct
