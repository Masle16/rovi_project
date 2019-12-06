cmake_minimum_required(VERSION 3.1)
set (CMAKE_CXX_STANDARD 11)
# Project name
project( image_based )

# Used for: Resolve absolute path names
set(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

set(RW_ROOT /home/mathi/RobWork/RobWork)
set(RWSTUDIO_ROOT /home/mathi/RobWork/RobWorkStudio)

set(CMAKE_BUILD_TYPE Release)
set(RobWork_DIR ${RW_ROOT}/cmake)
set(RobWorkStudio_DIR ${RWSTUDIO_ROOT}/cmake)

find_package(RobWork REQUIRED)
find_package(RobWorkStudio REQUIRED)
find_package(OpenCV REQUIRED EXACT 4.1.2 )
find_package(PCL 1.4 REQUIRED)
find_package(COVIS HINTS /home/mathi/Documents/covis/build)
find_package(VTK REQUIRED NO_MODULE)

include(${VTK_USE_FILE})

include_directories(
    ${PCL_INCLUDE_DIRS}
    ${ROBWORK_INCLUDE_DIRS}
    ${ROBWORKSTUDIO_INCLUDE_DIRS}
    ${COVIS_INCLUDE_DIRS}
)
link_directories(
    ${PCL_LIBRARY_DIRS}
    ${ROBWORK_LIBRARY_DIRS}
    ${ROBWORKSTUDIO_LIBRARY_DIRS}
)
add_definitions(
    ${PCL_DEFINITIONS}
    ${COVIS_DEFINITIONS}
)

add_executable(
    image_based
    main.cpp
)
target_link_libraries(
    image_based
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    ${ROBWORK_LIBRARIES}
    ${ROBWORKSTUDIO_LIBRARIES}
    ${COVIS_LIBRARIES}
)
