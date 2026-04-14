#pragma once

#include "gtsam/nonlinear/NoiseModelFactorN.h"
#include "trajectory_optimization/gpmp_optimizer/models/wnoa_origin.hpp"

class GPPriorFactorOrigin
    : public gtsam::NoiseModelFactorN<gtsam::Vector4, gtsam::Vector4> {
 public:
  using Base = gtsam::NoiseModelFactorN<gtsam::Vector4, gtsam::Vector4>;
  GPPriorFactorOrigin(gtsam::Key key1, gtsam::Key key2, const double delta,
                      const double Qc)
      : Base(gtsam::noiseModel::Gaussian::Covariance(
                              calcQ(Qc * Eigen::Matrix2d::Identity(), delta)),
                          key1, key2),
        delta_(delta),
        phi_(calcPhi(2, delta)){};
  ~GPPriorFactorOrigin() = default;

  gtsam::Vector evaluateError(
      const gtsam::Vector4& x1, const gtsam::Vector4& x2,
      gtsam::Matrix* H1 = nullptr,
      gtsam::Matrix* H2 = nullptr) const override;

  void verbose() {}

 private:
  double delta_ = 0.0;
  gtsam::Matrix44 phi_;
};