cmake_minimum_required(VERSION 3.1)
set (CMAKE_CXX_STANDARD 11)
# Project name
project( simulated_depth_sensor )

# Used for: Resolve absolute path names
set(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

set(RW_ROOT /home/mathi/RobWork/RobWork)
set(RWSTUDIO_ROOT /home/mathi/RobWork/RobWorkStudio)

set(CMAKE_BUILD_TYPE Debug)
set(RobWork_DIR ${RW_ROOT}/cmake)
set(RobWorkStudio_DIR ${RWSTUDIO_ROOT}/cmake)

find_package(RobWork REQUIRED)
find_package(RobWorkStudio REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.4 REQUIRED)

include_directories(
    ${PCL_INCLUDE_DIRS}
    ${ROBWORK_INCLUDE_DIRS}
    ${ROBWORKSTUDIO_INCLUDE_DIRS}
)
link_directories(
    ${PCL_LIBRARY_DIRS}
    ${ROBWORK_LIBRARY_DIRS}
    ${ROBWORKSTUDIO_LIBRARY_DIRS}
)
add_definitions(${PCL_DEFINITIONS})

add_executable(
    simulated_depth_sensor
    main.cpp
    alignment.hpp
    util.hpp
)
target_link_libraries(
    simulated_depth_sensor
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    ${ROBWORK_LIBRARIES}
    ${ROBWORKSTUDIO_LIBRARIES}
)
