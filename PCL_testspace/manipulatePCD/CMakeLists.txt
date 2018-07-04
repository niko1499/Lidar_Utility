cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(manipulatePCD)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (centerPCD centerPCD.cpp)
target_link_libraries (centerPCD ${PCL_LIBRARIES})
add_executable (cutPCD cutPCD.cpp)
target_link_libraries (cutPCD ${PCL_LIBRARIES})
add_executable (mergePCD mergePCD.cpp)
target_link_libraries (mergePCD ${PCL_LIBRARIES})
