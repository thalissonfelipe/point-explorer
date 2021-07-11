cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(point_explorer)

find_package(PCL 1.7.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

link_libraries(stdc++fs)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/build")

add_executable (
  point_explorer 
  GeometricFeatures.cpp
  Computation.cpp
  ../include/GeometricFeatures.h
  ../include/Computation.h
  main.cpp
  )
target_link_libraries (point_explorer ${PCL_LIBRARIES})