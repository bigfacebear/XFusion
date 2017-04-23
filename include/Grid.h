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

//class Old_Grid {
//public:
//    // The container of voxels
//    Voxel*** voxels;  // TODO: make a container for it
//
//    Old_Grid(int K);
//    ~Old_Grid();
//
//    int getK() { return K; }
//
//    inline Voxel* getVoxelByCoordinate(double x, double y, double z);
//    inline Voxel* getVoxelByCoordinate(cv::Vec3d v);
//    bool getColorAndPointByRay(cv::Vec3d p, cv::Vec3d v, cv::Vec3b &C, cv::Mat &worldP);
//
//private:
//    int K;
//
//    double min(double a, double b) { return (a > b) ? b : a; }
//    double max(double a, double b) { return (a > b) ? a : b; }
//};

class Grid {
public:
    // voxel information
    GLbyte ****C; // RGBA, 4 channels for the convenience of passing data to OpenGL Shader
    GLfloat ***S;  // The distance to the surface from the voxel
    int ***W;  // voxel's weight when in fusing stage

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

private:
    int K;  // dimension of the grid
};

#endif