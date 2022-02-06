cmake_minimum_required(VERSION 2.8)
set(CMAKE_VERBOSE_MAKEFILE 1) # to see all make process

project( pair_feature_colored_icp )

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable ( pair_feature_colored_icp src/icp_feature_registration.cpp src/visualize_correspondences.cpp )
target_link_libraries ( pair_feature_colored_icp ${PCL_LIBRARIES})
