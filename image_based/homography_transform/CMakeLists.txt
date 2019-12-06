CMAKE_MINIMUM_REQUIRED(VERSION 2.8.0)

project(homograpy)

find_package(OpenCV 4.1.2 REQUIRED)

add_executable(homograpy main.cpp util.hpp)
target_link_libraries(homograpy ${OpenCV_LIBS})
