add_library(map_manager STATIC
  dense_elevation_map.cc
)
set_target_properties(map_manager PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(map_manager PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/..
)
