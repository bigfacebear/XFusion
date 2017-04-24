#pragma once

#ifndef XFUSION_GRID_H
#define XFUSION_GRID_H

#include <GL/glew.h>
#include <opencv2/opencv.hpp>
#include <vector>

#include "definition.h"

class Grid {
public:

    Grid(int K);
    ~Grid();

    int getK() { return K; }

    void getImageAndDepthFromViewPoint(
            // input
            PoseMatx Tv, // camera pose
            cv::Mat intrinsic_matrix, // camera's intrinsic parameters
            // output
            cv::Mat &IM, // RGBA image, data in uchar
            cv::Mat &DM,  // depth map, data in GLfloat
            cv::Mat &WM  // weight map, data in GLfloat
    );

    GLbyte* getC(int x, int y, int z) {
        return &C[4 * getIndex(x, y, z)];
    }

    GLfloat& getS(int x, int y, int z) {
        return S[getIndex(x, y, z)];
    }

    GLfloat& getW(int x, int y, int z) {
        return W[getIndex(x, y, z)];
    }

private:
    int K;  // dimension of the grid

    GLbyte *C;
    GLfloat *S;
    GLfloat *W;

    int getIndex(int x, int y, int z) {
        return K * K * x + K * y + z;
    }
};

#endif