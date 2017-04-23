cmake_minimum_required(VERSION 3.7)
project(XFusion)

set(CMAKE_CXX_STANDARD 11)

# OpenGL
find_package(OpenGL REQUIRED)
if(OPENGL_FOUND)
    message("   Found OpenGL - include: ${OPENGL_INCLUDE_DIRS}, lib: ${OPENGL_LIBRARIES}")
else(OPENGL_FOUND)
    message("   Failed to find OpenGL.")
endif(OPENGL_FOUND)

# GLEW
find_package(GLEW REQUIRED)
if(GLEW_FOUND)
    message("   Found GLEW - include: ${GLEW_INCLUDE_DIRS}, lib: ${GLEW_LIBRARIES}")
else(GLEW_FOUND)
    message("   Failed to find GLEW.")
endif(GLEW_FOUND)

# GLFW3
find_package(glfw3 REQUIRED)
if(glfw3_FOUND)
    message("   Found GLFW3 - include: ${GLFW3_INCLUDE_DIR}, lib: ${GLFW3_LIBRARY}")
else(glfw3_FOUND)
    message("   Failed to find GLFW3.")
endif(glfw3_FOUND)

# GLM
find_path(GLM_INCLUDE_DIR glm)
message("   Found GLM - include: ${GLM_INCLUDE_DIR}")

# OpenCV
find_package(OpenCV REQUIRED)
if (OpenCV_FOUND)
    message("   Found OpenCV - include: ${OpenCV_INCLUDE_DIRS}, lib: ${OpenCV_LIBRARIES}")
else(OpenCV_FOUND)
    message("   Failed to find OpenCV")
endif ()


# Ceres-Solver
find_package(Ceres REQUIRED)
if (Ceres_FOUND)
    message("   Found Ceres - include: ${CERES_INCLUDE_DIRS}, lib: ${CERES_LIBRARIES}")
else (Ceres_FOUND)
    message("   Failed to find Ceres")
endif(Ceres_FOUND)

# Project headers
include_directories(
        ${PROJECT_SOURCE_DIR}/include
        ${OPENGL_INCLUDE_DIRS}
        ${GLEW_INCLUDE_DIRS}
        ${GLFW3_INCLUDE_DIR}
        ${GLM_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
)

set(SOURCE_DIR ${PROJECT_SOURCE_DIR}/src)
set(SOURCE_FILES
        ${SOURCE_DIR}/main.cpp
        ${SOURCE_DIR}/XFuser.cpp
        ${SOURCE_DIR}/IOProcessor.cpp
        ${SOURCE_DIR}/Grid.cpp
        ${SOURCE_DIR}/FramePool.cpp
        ${SOURCE_DIR}/utils.cpp)

add_executable(XFusion ${SOURCE_FILES})
target_link_libraries(XFusion
        ${OPENGL_LIBRARIES}
        ${GLEW_LIBRARIES}
        ${GLFW3_LIBRARY}
        ${OpenCV_LIBRARIES}
        ${CERES_LIBRARIES})
