cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

# c++0x compatibility
#ADD_DEFINITIONS("-std=c++0x")
rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

# finding required freetail source files
#rosbuild_find_ros_package(freetail_object_manipulation)
#if(DEFINED freetail_object_manipulation_PACKAGE_PATH)
#	message(STATUS "Found freetail library")
#	set(FREETAIL_PATH ${freetail_object_manipulation_PACKAGE_PATH})
#	set(FREETAIL_SRC ${freetail_object_manipulation_PACKAGE_PATH}/src)
#	set(FREETAIL_INCLUDE ${freetail_object_manipulation_PACKAGE_PATH}/include)
#	include_directories(${FREETAIL_PATH})
#	include_directories(${FREETAIL_INCLUDE})	
#	include_directories(${FREETAIL_SRC})
#else(DEFINED freetail_object_manipulation_PACKAGE_PATH)
#	message(FATAL_ERROR "Failed to find freetail library")
#endif(DEFINED freetail_object_manipulation_PACKAGE_PATH)

#rosbuild_add_library(RobotNavigator src/arm_navigators/RobotPickPlaceNavigator.cpp 
#${FREETAIL_SRC}/utils/CustomPlaceTester.cpp ${FREETAIL_SRC}/segmentation/SphereSegmentation.cpp)

rosbuild_add_library(${PROJECT_NAME} 
	src/arm_navigators/RobotPickPlaceNavigator.cpp 
	src/arm_navigators/SpherePickingRobotNavigator.cpp
	src/zone_selection/PickPlaceZoneSelector.cpp
	src/arm_navigators/AutomatedPickerRobotNavigator.cpp
	src/arm_navigators/SortClutterArmNavigator.cpp)

rosbuild_add_executable(mantis_pick_place_node src/nodes/pick_place_demo_node.cpp)
target_link_libraries(mantis_pick_place_node ${PROJECT_NAME})

rosbuild_add_executable(mantis_sphere_picking_node src/nodes/sphere_picking_demo_node.cpp)
target_link_libraries(mantis_sphere_picking_node ${PROJECT_NAME})

rosbuild_add_executable(test_zone_selector_node src/test/test_zone_selector_node.cpp)
target_link_libraries(test_zone_selector_node ${PROJECT_NAME})

rosbuild_add_executable(automated_pick_place_node src/nodes/automated_pick_place_node.cpp)
target_link_libraries(automated_pick_place_node ${PROJECT_NAME})

rosbuild_add_executable(dual_arm_supervisor_node src/nodes/dual_arm_supervisor_node.cpp)
target_link_libraries(dual_arm_supervisor_node ${PROJECT_NAME})

rosbuild_add_executable(sort_clutter_arm_node src/nodes/sort_clutter_arm_node.cpp)
target_link_libraries(sort_clutter_arm_node ${PROJECT_NAME})

rosbuild_add_executable(concurrent_arm_move_supervisor_node src/nodes/arm_move_supervisor_node.cpp)
target_link_libraries(concurrent_arm_move_supervisor_node ${PROJECT_NAME})

#rosbuild_add_executable(sensor_data_redirect_node src/nodes/sensor_data_redirect.cpp)
#rosbuild_add_executable(test_grasp_action_server src/test/test_grasp_action_server_node.cpp)
