find_package(osqp REQUIRED)

add_library(common_smoothing STATIC
  affine_constraint.cc
  discrete_points_math.cc
  osqp_spline1d_solver.cc
  osqp_spline2d_solver.cc
  polynomialxd.cc
  spline1d_constraint.cc
  spline1d_kernel_helper.cc
  spline1d_kernel.cc
  spline1d_seg.cc
  spline1d.cc
  spline2d_constraint.cc
  spline2d_kernel.cc
  spline2d_seg.cc
  spline2d.cc
  solver/osqp/osqp_interface.cc
  utils/math.cc
)
set_target_properties(common_smoothing PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(common_smoothing PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/../..
)
target_link_libraries(common_smoothing PUBLIC
  osqp::osqp
)
