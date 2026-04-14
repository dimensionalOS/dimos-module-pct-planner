add_library(pct_tomography STATIC
  tomogram.cpp
  kernels.cpp
)
set_target_properties(pct_tomography PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(pct_tomography PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/..
  ${EIGEN3_INCLUDE_DIR}
)
