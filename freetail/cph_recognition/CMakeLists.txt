cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# we need FindFLANN.cmake 
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

project(cph_classifier)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
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
if(HDF5_FOUND)

  find_package(Flann)
  include_directories(${FLANN_INCLUDE_DIRS})

  include_directories(${HDF5_INCLUDE_DIR})

rosbuild_add_executable(generate_features src/generate_features.cpp)
target_link_libraries(generate_features ${PCL_LIBRARIES} 
                                        boost_system boost_filesystem ${Boost_LIBRARIES} ${HDF5_hdf5_LIBRARY}
)
 
rosbuild_add_executable(cph_recognition src/cph_recognition.cpp)
target_link_libraries(cph_recognition ${PCL_LIBRARIES} 
                                        boost_system boost_filesystem ${Boost_LIBRARIES} ${HDF5_hdf5_LIBRARY}
)
rosbuild_add_executable(test_client src/test_client.cpp)
target_link_libraries(test_client ${PCL_LIBRARIES} 
                                        boost_system boost_filesystem ${Boost_LIBRARIES} ${HDF5_hdf5_LIBRARY}
)
rosbuild_add_executable(collect_data src/collect_data.cpp)
target_link_libraries(collect_data ${PCL_LIBRARIES} 
                                        boost_system boost_filesystem ${Boost_LIBRARIES} ${HDF5_hdf5_LIBRARY}
) 
endif(HDF5_FOUND)


