//
// Created by bfb on 17-4-23.
//

#ifndef XFUSION_DEFINITION_H
#define XFUSION_DEFINITION_H

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <opencv2/opencv.hpp>
#include <ceres/ceres.h>

#include "Shader.h"

typedef cv::Mat PoseMatx;

const GLuint FRAME_WIDTH = 320;
const GLuint FRAME_HEIGHT = 240;
const cv::Size IMG_SIZE(FRAME_WIDTH, FRAME_HEIGHT);

void gpuRun(Shader &shader);

void getShaderBufferObjectDataToImage(GLuint buffer, size_t pixelSize, cv::Mat& img);

#endif //XFUSION_DEFINITION_H
