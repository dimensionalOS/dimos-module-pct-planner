#pragma once

#include <memory>

#include "gtsam/nonlinear/NoiseModelFactorN.h"
#include "map_manager/dense_elevation_map.h"

class GPObstacleFactorWnoa : public gtsam::NoiseModelFactorN<gtsam::Vector4> {
 public:
  using Base = gtsam::NoiseModelFactorN<gtsam::Vector4>;
  GPObstacleFactorWnoa(gtsam::Key key, std::shared_ptr<DenseElevationMap> map,
                       int current_layer, const double height_hint,
                       const double q_cost, const double cost_threshold,
                       bool verbose = false)
      : Base(gtsam::noiseModel::Isotropic::Sigma(1, q_cost), key),
        current_layer_(current_layer),
        height_hint_(height_hint),
        cost_threshold_(cost_threshold),
        map_(map),
        verbose_(verbose) {}
  ~GPObstacleFactorWnoa() = default;

  int GetNodeLayer() const { return current_layer_; }

  gtsam::Vector evaluateError(
      const gtsam::Vector4& x1,
      gtsam::Matrix* H1 = nullptr) const override;

  void verbose() { verbose_ = true; }

 private:
  bool verbose_ = false;
  mutable double height_hint_ = 0.0;
  mutable int current_layer_ = 0;
  double cost_threshold_ = 0.0;
  std::shared_ptr<DenseElevationMap> map_;
};