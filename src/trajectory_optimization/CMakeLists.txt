add_library(gpmp_optimizer STATIC
  gpmp_optimizer/factors/gp_prior_factor.cc
  gpmp_optimizer/factors/gp_obstacle_factor.cc
  gpmp_optimizer/factors/gp_interpolate_obstacle_factor.cc
  gpmp_optimizer/factors/gp_heading_rate_factor.cc
  gpmp_optimizer/factors/gp_interpolate_heading_rate_factor.cc
  gpmp_optimizer/factors_wnoa/gp_prior_factor.cc
  gpmp_optimizer/factors_wnoa/gp_obstacle_factor.cc
  gpmp_optimizer/factors_wnoa/gp_interpolate_obstacle_factor.cc
  gpmp_optimizer/factors_origin/gp_prior_factor.cc
  gpmp_optimizer/interpolator/wnoj_trajectory_interpolator.cc
  gpmp_optimizer/interpolator/wnoa_trajectory_interpolator.cc
  gpmp_optimizer/gpmp_optimizer.cc
  gpmp_optimizer/gpmp_optimizer_wnoa.cc
  height_smoother/height_smoother.cc
)
set_target_properties(gpmp_optimizer PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(gpmp_optimizer PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/..
  ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(gpmp_optimizer PUBLIC
  gtsam
  map_manager
  common_smoothing
)
