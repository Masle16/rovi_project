# Test CMake version
CMAKE_MINIMUM_REQUIRED(VERSION 2.6.0)

# The name of the project
PROJECT(sparse_stereo)
MESSAGE(${PROJECT_NAME} ":")

# Used to resolve absolute path names
SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

# Use c++11
SET(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

# Set the RobWork root (edit in .bashrc if necessary)
SET(RW_ROOT /home/mathi/RobWork/RobWork)

# Verbose mode prints more info when doing make
set(CMAKE_VERBOSE_MAKEFILE false)

# Set build type to release
SET(CMAKE_BUILD_TYPE Release)
MESSAGE("-- Build type: " ${CMAKE_BUILD_TYPE})

# Use RobWork
SET(RobWork_DIR ${RW_ROOT}/cmake)
FIND_PACKAGE(RobWork REQUIRED)
find_package(OpenCV REQUIRED)
INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} )

# Add the source files to build
SET(SrcFiles
    main.cpp
    util.hpp
#    mesh.hpp
#    csv_reader.hpp
#    pnp_problem.hpp
)

# The shared library to build:
ADD_EXECUTABLE(sparse_stereo ${SrcFiles})
TARGET_LINK_LIBRARIES(sparse_stereo ${ROBWORK_LIBRARIES} ${OpenCV_LIBS})

MESSAGE(${PROJECT_NAME} " done!")
