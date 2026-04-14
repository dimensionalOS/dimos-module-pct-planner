add_library(a_star_search STATIC
  a_star_search.cc
)
set_target_properties(a_star_search PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(a_star_search PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/..
)
