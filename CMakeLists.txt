cmake_minimum_required(VERSION 3.7)
project(XFusion)

set(CMAKE_CXX_STANDARD 11)

# OpenCV
if(APPLE)
    find_package(OpenCV HINTS /usr/local/opt/opencv3 REQUIRED)
else()
    find_package(OpenCV REQUIRED)
endif()
include_directories(${OpenCV_INCLUDE_DIRS})

# Ceres-Solver
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

# Project headers
include_directories(${PROJECT_SOURCE_DIR}/include)

set(SOURCE_DIR ${PROJECT_SOURCE_DIR}/src)
set(SOURCE_FILES
        ${SOURCE_DIR}/main.cpp
        ${SOURCE_DIR}/XFusion.cpp
        ${SOURCE_DIR}/IOProcessor.cpp
        ${SOURCE_DIR}/Grid.cpp
        ${SOURCE_DIR}/FramePool.cpp
)

add_executable(XFusion ${SOURCE_FILES})
target_link_libraries(XFusion ${OpenCV_LIBRARIES} ${CERES_LIBRARIES})
