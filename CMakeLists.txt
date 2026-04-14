cmake_minimum_required(VERSION 3.14)
project(pct_planner CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include(FetchContent)

FetchContent_Declare(dimos_lcm
  GIT_REPOSITORY https://github.com/dimensionalOS/dimos-lcm.git
  GIT_TAG main
  GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(dimos_lcm)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LCM REQUIRED lcm)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(osqp REQUIRED)

add_subdirectory(src/common/smoothing)
add_subdirectory(src/a_star)
add_subdirectory(src/map_manager)
add_subdirectory(src/trajectory_optimization)
add_subdirectory(src/ele_planner)
add_subdirectory(src/tomography)
add_subdirectory(src/planner)

add_executable(pct_planner main.cpp)
target_include_directories(pct_planner PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/common
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${dimos_lcm_SOURCE_DIR}/generated/cpp_lcm_msgs
  ${LCM_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(pct_planner PRIVATE
  ${LCM_LIBRARIES}
  pct_planner_lib
  pct_tomography
  ele_planner_lib
)
target_link_directories(pct_planner PRIVATE ${LCM_LIBRARY_DIRS})

install(TARGETS pct_planner DESTINATION bin)
