#pragma once

#include <opencv2/opencv.hpp>
#include <string>

#include "Grid.h"
#include "FramePool.h"
#include "IOProcessor.h"

typedef cv::Mat PoseMatx;

class XFuser {
public:
    XFuser();
    XFuser(cv::Mat intrinsic_matrix);

    void load(const std::string &videoName);
    void load(const int cameraHandler);

    void fuse();

    static void bootstrap(FramePool &framePool, Grid &grid, IOProcessor &ioProcessor);
    /*Estimate the camera pose of the inputing image
    * Ii is the inputing image
    * Tv is the positon of the previous frame
    * grid is the grid that holds the model
    */
    static PoseMatx poseEstimate(cv::Mat &Ii, PoseMatx &Tv, Grid &grid, cv::Mat intrinsic_matrix);
    
    static cv::Mat depthEstimate(const Frame &Ii, const Frame &Ik);

    static void fuseToGrid(Grid &grid, PoseMatx &T, cv::Mat I, cv::Mat dmap, cv::Mat intrinsic_matrix);

    static void raycastingFromTv(PoseMatx &Tv, Grid &grid, cv::Size size, cv::Mat intrinsic_matrix, cv::Mat &IM, cv::Mat &DM);

private:
    Grid grid;
    FramePool framePool;
    //Initializer initializer;
    IOProcessor ioProcessor;

    cv::Mat intrinsic_matrix;
    //PoseEstimator poseEstimator;
    //DepthEstimator depthEstimator;

};