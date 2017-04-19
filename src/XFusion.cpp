#include "XFusion.h"

#include <vector>

XFuser::XFuser(): grid(256), framePool(40) {
}

XFuser::XFuser(cv::Mat intrinsic_matrix): grid(256), framePool(40), intrinsic_matrix(intrinsic_matrix) {

}

void XFuser::load(const std::string &videoName) {
    ioProcessor.loadVideo(videoName);
}

void XFuser::load(const int cameraHandler)
{
}

void XFuser::fuse() {

    bootstrap(framePool, grid, ioProcessor);

    for (cv::Mat Iv = ioProcessor.get(); !Iv.empty(); Iv = ioProcessor.get()) {
        // get a new image from IO, and estimate the pose of the camera
        PoseMatx Tv = framePool.getLatestFrame().T;
        //PoseMatx Tvl = poseEstimator.poseEstimate(Iv, Tv, grid);
        PoseMatx Tvl = poseEstimate(Iv, Tv, grid, intrinsic_matrix);
        PoseMatx T = Tvl * Tv;
        Frame newFrame(Iv, T);

        // Select a keyframe from the frame pool
        Frame &keyFrame = framePool.getKeyFrame(newFrame, intrinsic_matrix);
        // Update the frame pool
        framePool.addFrame(newFrame);

        // Estimate depth map
        // cv::Mat dmap = depthEstimator.depthEstimate(newFrame, keyframe);
        cv::Mat dmap = depthEstimate(newFrame, keyFrame);

        fuseToGrid(grid, T, Iv, dmap, intrinsic_matrix);

        // TODO: live feedback during processing
    }

    // TODO: display the result
    
}

void XFuser::bootstrap(FramePool & framePool, Grid & grid, IOProcessor & ioProcessor)
{
}

PoseMatx XFuser::poseEstimate(cv::Mat &Ii, PoseMatx &Tv, Grid &grid, cv::Mat intrinsic_matrix) {
    cv::Mat IM(Ii.size(), CV_8UC3, cv::Scalar::all(0));
    cv::Mat DM(Ii.size(), CV_64FC1, cv::Scalar::all(0));
    raycastingFromTv(Tv, grid, Ii.size(), intrinsic_matrix, IM, DM);
    cv::imshow("IM", IM);
    cv::Mat DM_show(Ii.size(), CV_8UC1, cv::Scalar::all(0));
    cv::imshow("DM", DM_show);
    /*for (int v = 0; v < IM.rows; v++) {
        for (int u = 0; u < IM.cols; u++) {
            
        }
    }*/
    return PoseMatx();
}

cv::Mat XFuser::depthEstimate(const Frame & Ii, const Frame & Ik)
{
    return cv::Mat();
}

void XFuser::fuseToGrid(Grid & grid, PoseMatx & T, cv::Mat I, cv::Mat dmap, cv::Mat intrinsic_matrix) {
    int K = grid.getK();
    for (int x = 0; x < K; x++) {
        for (int y = 0; y < K; y++) {
            for (int z = 0; z < K; z++) {
                cv::Vec4d p(x - K / 2, y - K / 2, z, 1);  // voxel's world coordinate
                cv::Vec4d q = cv::Mat(T * cv::Mat(p));  // voxel's camera coordinate
                cv::Vec3d u_homo = cv::Mat(intrinsic_matrix * cv::Mat(cv::Mat(q), cv::Rect(0, 0, 1, 3)));
                int u = (int)(u_homo[0] / u_homo[2]);  // voxel's image coordinate.u
                int v = (int)(u_homo[1] / u_homo[2]);  // voxel's image coordinate.v
                if (u < 0 || v < 0 || u >= I.cols || v >= I.rows) {
                    continue;  // this voxel is not on the image
                }
                double s = dmap.at<double>(v, u) - q[3] / q[4]; // s = D(u,v) - q.z
                if (s < -8) {
                    continue;  // only update visible voxels
                }
                if (s > 8) {
                    s = 8;  // truncate s to 8
                }
                cv::Mat c = cv::Mat(I.at<cv::Vec3b>(v, u));
                c.convertTo(c, CV_64FC3);
                Voxel &voxel = grid.voxels[x][y][z];
                voxel.S = (voxel.S * voxel.W + s) / (voxel.W + 1);
                cv::Mat tmp(voxel.C);
                tmp.convertTo(tmp, CV_64FC3);
                tmp = (tmp * voxel.W + c) / (voxel.W + 1);
                tmp.convertTo(tmp, CV_8UC3);
                voxel.C = tmp;
                voxel.W = voxel.W == 50 ? 50 : voxel.W + 1;
            }
        }
    }
}

void XFuser::raycastingFromTv(PoseMatx & Tv, Grid & grid, cv::Size size, cv::Mat intrinsic_matrix, cv::Mat & IM, cv::Mat & DM) {
    IM = cv::Mat(size, CV_8UC3);
    DM = cv::Mat(size, CV_64FC1, cv::Scalar::all(-1));

    cv::Mat intrinsic_matrix_inv = intrinsic_matrix.inv();

    cv::Mat R_Camera(Tv, cv::Rect(0, 0, 3, 3));
    cv::Mat R_Camera_inv = R_Camera.inv();
    cv::Mat T_Camera = -R_Camera_inv * cv::Mat(Tv, cv::Rect(3, 0, 1, 3));

    std::cout << T_Camera << std::endl;

    for (int u = 0; u < size.width; u++) {
        for (int v = 0; v < size.height; v++) {
            cv::Mat p(cv::Vec3d(u, v, 1));

            p = intrinsic_matrix_inv * p;
            cv::Mat rayDir = R_Camera_inv * p;

            // raycast
            cv::Vec3b color;
            cv::Mat worldP;
            bool if_hit = grid.getColorAndPointByRay(T_Camera, rayDir, color, worldP);
            if (if_hit) {
                cv::Vec3d cameraP = cv::Mat(R_Camera * worldP - T_Camera);
                IM.at<cv::Vec3b>(v, u) = color;
                DM.at<double>(v, u) = cameraP[2];
            }
            else {
                IM.at<cv::Vec3b>(v, u) = cv::Vec3b::all(0);
                DM.at<double>(v, u) = INFINITY;
            }
        }
    }
}

