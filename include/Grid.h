#pragma once

#ifndef XFUSION_GRID_H
#define XFUSION_GRID_H

#include <GL/glew.h>
#include <opencv2/opencv.hpp>
#include <vector>

#include "definition.h"

//class Voxel {
//public:
//
//    Voxel() :C(0, 0, 0), S(INFINITY), W(0) {}  // TODO: The initial value of S ?
//
//    cv::Vec3b C;
//    double S;
//    int W;
//};
//
//class Grid {
//public:
//    // The container of voxels
//    Voxel*** voxels;  // TODO: make a container for it
//
//    Grid(int K);
//    ~Grid();
//
//    int getK() { return K; }
//
//    inline Voxel* getVoxelByCoordinate(float x, float y, float z);
//    inline Voxel* getVoxelByCoordinate(cv::Vec3f v);
//    bool getColorAndPointByRay(cv::Vec3f p, cv::Vec3f v, cv::Vec3b &C, cv::Mat &worldP);
//
//private:
//    int K;
//
//    float min(float a, float b) { return (a > b) ? b : a; }
//    float max(float a, float b) { return (a > b) ? a : b; }
//};

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
            cv::Mat &DM  // depth map, data in GLfloat
    );

    GLbyte* getC(int x, int y, int z) {
        return &C[4 * getIndex(x, y, z)];
    }

    GLfloat& getS(int x, int y, int z) {
        return S[getIndex(x, y, z)];
    }

    int& getW(int x, int y, int z) {
        return W[getIndex(x, y, z)];
    }

private:
    int K;  // dimension of the grid

    GLbyte *C;
    GLfloat *S;
    int *W;

    int getIndex(int x, int y, int z) {
        return K * K * x + K * y + z;
    }
};

#endif