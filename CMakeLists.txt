cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(plytopcd)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (color color.cpp)

target_link_libraries (color ${PCL_LIBRARIES})

add_executable (clustering clustering.cpp)

target_link_libraries (clustering ${PCL_LIBRARIES})

add_executable (pose_estimate pose_estimate.cpp)

target_link_libraries (pose_estimate ${PCL_LIBRARIES})







