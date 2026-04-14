#pragma once

#include "gtsam/nonlinear/NoiseModelFactor.h"
#include "trajectory_optimization/gpmp_optimizer/models/wnoa.hpp"

class GPPriorFactorWnoa
    : public gtsam::NoiseModelFactorN<gtsam::Vector4, gtsam::Vector4> {
 public:
  using Base = gtsam::NoiseModelFactorN<gtsam::Vector4, gtsam::Vector4>;
  GPPriorFactorWnoa(gtsam::Key key1, gtsam::Key key2, const double delta,
                const double Qc)
      : Base(gtsam::noiseModel::Gaussian::Covariance(
                              WhiteNoiseOnAcceleration2D::Q(Qc, delta)),
                          key1, key2),
        delta_(delta),
        phi_(WhiteNoiseOnAcceleration2D::Phi(delta)){};
  ~GPPriorFactorWnoa() = default;

  gtsam::Vector evaluateError(
      const gtsam::Vector4& x1, const gtsam::Vector4& x2,
      gtsam::Matrix* H1 = nullptr,
      gtsam::Matrix* H2 = nullptr) const override;

  void verbose() {}

 private:
  double delta_ = 0.0;
  gtsam::Matrix44 phi_;
};