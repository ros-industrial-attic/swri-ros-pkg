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

rosbuild_add_library(ManipulationDemo src/demos/SimpleManipulationDemo.cpp 
	src/utils/CustomPlaceTester.cpp 
	src/segmentation/SphereSegmentation.cpp
	src/arm_navigators/RobotGripperNavigator.cpp)

rosbuild_add_executable(fast_demo_app_swri src/demos/fast_demo_app_swri.cpp)

rosbuild_add_executable(manipulation_demo_node src/demos/manipulation_demo_node.cpp)
target_link_libraries(manipulation_demo_node ManipulationDemo)

rosbuild_add_executable(ball_picking_demo_node src/demos/BallPickingDemoNode.cpp)
target_link_libraries(ball_picking_demo_node ManipulationDemo)

rosbuild_add_executable(test_recognition_node src/tests/test_recognition_node.cpp)
target_link_libraries(test_recognition_node ManipulationDemo)

rosbuild_add_executable(test_sphere_segmentation 
	src/tests/test_sphere_segmentation.cpp 
	src/segmentation/SphereSegmentation.cpp)
	
rosbuild_add_executable(robot_pick_shear_node src/demos/robot_pick_shear_node.cpp)
target_link_libraries(robot_pick_shear_node ManipulationDemo)
	