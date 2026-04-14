add_library(pct_planner_lib STATIC
  pct_planner.cpp
  path_utils.cpp
)
set_target_properties(pct_planner_lib PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(pct_planner_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/..
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(pct_planner_lib PUBLIC
  pct_tomography
  ele_planner_lib
  a_star_search
  map_manager
  gpmp_optimizer
)
