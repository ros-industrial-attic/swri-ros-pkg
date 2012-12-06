cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

# The definition is copied from the CMakeList for the simple_message package.
add_definitions(-DROS=1)  #build using ROS libraries

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
target_link_libraries(robot_state simple_message)

rosbuild_add_library(industrial_robot_client
  src/joint_relay_handler.cpp
  src/joint_trajectory_downloader.cpp
  src/joint_trajectory_interface.cpp
  src/joint_trajectory_streamer.cpp
  src/robot_state_interface.cpp)

rosbuild_add_executable(robot_state
  src/generic_robot_state_node.cpp)
target_link_libraries(robot_state industrial_robot_client)

rosbuild_add_executable(motion_streaming_interface
  src/generic_joint_streamer_node.cpp)
target_link_libraries(motion_streaming_interface industrial_robot_client)
rosbuild_link_boost(motion_streaming_interface thread)
target_link_libraries(motion_streaming simple_message)

rosbuild_add_executable(motion_download_interface
  src/generic_joint_downloader_node.cpp)
target_link_libraries(motion_download_interface industrial_robot_client)
target_link_libraries(motion_download_interface simple_message)

rosbuild_add_executable(joint_trajectory_action src/joint_trajectory_action.cpp)
rosbuild_link_boost(joint_trajectory_action thread)
#target_link_libraries(example ${PROJECT_NAME})
