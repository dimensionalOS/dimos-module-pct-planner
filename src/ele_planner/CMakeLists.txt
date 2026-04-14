add_library(ele_planner_lib STATIC
  offline_ele_planner.cc
)
set_target_properties(ele_planner_lib PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(ele_planner_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/..
)
target_link_libraries(ele_planner_lib PUBLIC
  a_star_search
  gpmp_optimizer
  map_manager
)
