cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

include_directories(${PCL_INCLUDE_DIRS})
link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions(${PCL_DEFINITIONS})

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

find_package(HDF5)
find_package(Flann)
include_directories(${FLANN_INCLUDE_DIRS})
#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)

# rosbuild_add_executable(vfh_recognition_node src/vfh_recognition_node.cpp
# 	src/SupportClasses.cpp 
# 	src/template_alignment.cpp
# 	src/euclidean_segmentation.cpp)
# target_link_libraries(vfh_recognition_node ${PCL_LIBRARIES} 
#                                       boost_system boost_filesystem  ${Boost_LIBRARIES} ${HDF5_hdf5_LIBRARY}
# )

rosbuild_add_executable(test_client src/test_client.cpp)
target_link_libraries(test_client ${PCL_LIBRARIES} 
                                        ${Boost_LIBRARIES} 
                                       )
rosbuild_add_executable(extract_vfh src/extract_vfh.cpp)
target_link_libraries(extract_vfh boost_system boost_filesystem ${PCL_LIBRARIES})
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_executable(vfh_recognition_configurable src/vfh_recognition_configurable.cpp 
	src/SupportClasses.cpp 
	src/template_alignment.cpp
	src/euclidean_segmentation.cpp)
target_link_libraries(vfh_recognition_configurable ${PCL_LIBRARIES}
	boost_system 
	boost_filesystem 
	${Boost_LIBRARIES}
	${HDF5_hdf5_LIBRARY}) 

rosbuild_add_executable(vfh_segmentation_node src/vfh_segmentation_node.cpp 
	src/SupportClasses.cpp 
	src/euclidean_segmentation.cpp)
target_link_libraries(vfh_segmentation_node ${PCL_LIBRARIES}
	boost_system 
	boost_filesystem 
	${Boost_LIBRARIES}
	${HDF5_hdf5_LIBRARY}) 