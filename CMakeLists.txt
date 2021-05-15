cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")
set (CMAKE_CXX_STANDARD 17)
project(playback)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories("src/quiz/cluster")
include_directories("src/quiz/ransac")
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")


add_executable (environment src/environment.cpp src/render/render.cpp
    src/processPointClouds.cpp src/ransac3d.h)
target_link_libraries (environment ${PCL_LIBRARIES})

#add_executable (quizCluster )


