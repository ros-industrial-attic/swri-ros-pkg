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

# The simple_message library is designed to cross compile on Ubuntu
# and various robot controlers.  This requires conditionally compiling
# certain functions and headers.  The definition below enables compiling
# for a ROS node.
add_definitions(-DROS=1)  #build using ROS libraryes

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/byte_array.cpp
#					src/simple_message.cpp)
rosbuild_add_library(simple_message 
					src/byte_array.cpp
					src/simple_message.cpp
					src/smpl_msg_connection.cpp
					src/udp_socket.cpp
					src/tcp_socket.cpp
					src/message_handler.cpp
					src/message_manager.cpp
					src/ping_handler.cpp
					src/ping_message.cpp
					)
					
rosbuild_add_gtest(utest test/utest.cpp
					src/byte_array.cpp
					src/simple_message.cpp
					src/smpl_msg_connection.cpp
					src/udp_socket.cpp
					src/tcp_socket.cpp
					src/message_handler.cpp
					src/message_manager.cpp
					src/ping_handler.cpp
					src/ping_message.cpp
					)
					
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
