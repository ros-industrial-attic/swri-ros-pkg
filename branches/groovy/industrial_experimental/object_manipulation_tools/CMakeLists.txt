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
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_library(object_manipulation_tools src/controller_utils/GraspPoseControllerHandler.cpp
	src/controller_utils/SimulatedController.cpp
	src/manipulation_utils/GraspSequenceValidator.cpp
	src/manipulation_utils/PlaceSequenceValidator.cpp
	src/grasp_planners/OverheadGraspPlanner.cpp
	src/services/GraspPlannerServer.cpp
	src/robot_navigators/RobotNavigator.cpp
	src/manipulation_utils/Utilities.cpp)

rosbuild_add_executable(grasp_action_service src/nodes/grasp_action_service_node.cpp)
target_link_libraries(grasp_action_service object_manipulation_tools)

rosbuild_add_executable(simulated_robot_controller src/nodes/simulated_robot_controller_node.cpp)
target_link_libraries(simulated_robot_controller object_manipulation_tools)

rosbuild_add_executable(overhead_grasp_planner_server src/nodes/overhead_grasp_service_node.cpp) 
target_link_libraries(overhead_grasp_planner_server object_manipulation_tools)

rosbuild_add_executable(joint_trajectory_action src/nodes/joint_trajectory_action.cpp) 