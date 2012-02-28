cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

include(FindPkgConfig)

set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

pkg_check_modules(JRL_WALKGEN REQUIRED jrl-walkgen)

# Main library.
rosbuild_add_library(${PROJECT_NAME}
  src/libjrl_walkgen_bridge/kajita.cpp)

rosbuild_add_compile_flags(${PROJECT_NAME} ${JRL_WALKGEN_CFLAGS})
rosbuild_add_link_flags(${PROJECT_NAME} ${JRL_WALKGEN_LDFLAGS})
