cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set (CMAKE_CXX_STANDARD 11)
project(velodyne_reader)

find_package(PCL 1.2 REQUIRED)
find_library (CNPY_LIBRARY cnpy)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (velodyne_reader velodyne_reader.cpp)
target_link_libraries (velodyne_reader ${PCL_LIBRARIES} ${CNPY_LIBRARY})
