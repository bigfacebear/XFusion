cmake_minimum_required(VERSION 3.7)
project(XFusion)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE "release")

find_package(OpenCV REQUIRED)
include_directories(
        OpenCV_INCLUDE_DIRS
        ${PROJECT_SOURCE_DIR}/include
)

set(SOURCE_DIR ${PROJECT_SOURCE_DIR}/src)
set(SOURCE_FILES
        ${SOURCE_DIR}/main.cpp
        ${SOURCE_DIR}/XFusion.cpp
        ${SOURCE_DIR}/IOProcessor.cpp
        ${SOURCE_DIR}/Grid.cpp
        ${SOURCE_DIR}/FramePool.cpp
)

add_executable(XFusion ${SOURCE_FILES})
target_link_libraries(XFusion ${OpenCV_LIBS})
