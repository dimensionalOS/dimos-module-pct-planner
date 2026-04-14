#pragma once

#include <memory>

#include "gtsam/nonlinear/NoiseModelFactor.h"
#include "map_manager/dense_elevation_map.h"

class GPHeadingRateFactor : public gtsam::NoiseModelFactorN<gtsam::Vector6> {
 public:
  using Base = gtsam::NoiseModelFactorN<gtsam::Vector6>;
  GPHeadingRateFactor(gtsam::Key key, const double max_heading_rate,
                      const double q_cost)
      : Base(gtsam::noiseModel::Isotropic::Sigma(1, q_cost), key),
        max_heading_rate_(max_heading_rate) {}
  ~GPHeadingRateFactor() = default;

  gtsam::Vector evaluateError(
      const gtsam::Vector6& x1,
      gtsam::Matrix* H1 = nullptr) const override;

 private:
  double max_heading_rate_ = 0.5;
};