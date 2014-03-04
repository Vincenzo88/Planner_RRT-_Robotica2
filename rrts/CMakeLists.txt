cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries

# for run time
#set(ROS_BUILD_TYPE Release)
#set(CMAKE_CXX_FLAGS "-std=c++0x")

# for debug
set(ROS_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "-std=c++0x -ggdb")

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#rosbuild_find_ros_package(dynamic_reconfigure)
#include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
#gencfg()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/dubins.cpp src/system.cpp src/kdtree.c)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

#rosbuild_add_executable(rrts_test src/test_main.cpp)
#rosbuild_add_executable(rrts_node src/rrts_node.cpp)
#rosbuild_add_executable(rrts_node_route src/rrts_node_route.cpp)
rosbuild_add_executable(rrts_global_map src/rrt_node_vincenzo.cpp)
target_link_libraries(rrts_global_map rrts_core)
#target_link_libraries(rrts_node_route rrts_core)
#target_link_libraries(rrts_test rrts_core)
