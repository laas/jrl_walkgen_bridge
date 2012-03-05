cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

include(FindPkgConfig)

set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_boost_directories()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIR})

pkg_check_modules(JRL_WALKGEN REQUIRED jrl-walkgen)

include_directories(include)

# Main library.
rosbuild_add_library(${PROJECT_NAME}
  src/libjrl_walkgen_bridge/kajita.cpp
  src/libjrl_walkgen_bridge/morisawa.cpp)

rosbuild_add_compile_flags(${PROJECT_NAME} ${JRL_WALKGEN_CFLAGS})
rosbuild_add_link_flags(${PROJECT_NAME} ${JRL_WALKGEN_LDFLAGS})

# Trajectory server.
rosbuild_add_executable(trajectory_server
  src/nodes/trajectory_server.cpp)
rosbuild_add_compile_flags(trajectory_server ${JRL_WALKGEN_CFLAGS})
rosbuild_add_link_flags(trajectory_server ${JRL_WALKGEN_LDFLAGS})
target_link_libraries(trajectory_server ${PROJECT_NAME})
