cmake_minimum_required(VERSION 3.1)
set (CMAKE_CXX_STANDARD 11)
project( dense_stereo )
find_package(OpenCV REQUIRED)
find_package(PCL 1.4 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable( dense_stereo main.cpp )
target_link_libraries(dense_stereo ${PCL_LIBRARIES} ${OpenCV_LIBS})
