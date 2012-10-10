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

rosbuild_add_library(RobotNavigator src/arm_navigators/RobotPickPlaceNavigator.cpp)

rosbuild_add_executable(mantis_pick_place_node src/demos/pick_place_demo_node.cpp)
target_link_libraries(mantis_pick_place_node RobotNavigator)
