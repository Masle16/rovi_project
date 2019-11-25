cmake_minimum_required(VERSION 2.8)
set (CMAKE_CXX_STANDARD 11)
project( sparse_stereo )
find_package( OpenCV REQUIRED )

add_executable( sparse_stereo main.cpp )
target_link_libraries(sparse_stereo ${OpenCV_LIBS})

