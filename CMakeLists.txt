cmake_minimum_required(VERSION 3.7)
project(XFusion)

set(CMAKE_CXX_STANDARD 11)

# OpenCV
find_package(OpenCV REQUIRED)
message("   Found OpenCV - include: ${OpenCV_INCLUDE_DIRS}, lib: ${OpenCV_LIBRARIES}")

# Ceres-Solver
find_package(Ceres REQUIRED)
message("   Found Ceres - include: ${CERES_INCLUDE_DIRS}, lib: ${CERES_LIBRARIES}")

# Project headers
include_directories(
        ${PROJECT_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
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
target_link_libraries(XFusion ${OpenCV_LIBRARIES} ${CERES_LIBRARIES})
