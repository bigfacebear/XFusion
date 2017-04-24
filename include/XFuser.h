//
// Created by bfb on 17-4-21.
//

#ifndef XFUSION_XFUSER_H
#define XFUSION_XFUSER_H

#include "definition.h"
#include "Grid.h"
#include "FramePool.h"
#include "IOProcessor.h"
#include <ceres/rotation.h>
#include <ceres/cubic_interpolation.h>

class XFuser {
public:
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

private:
    Grid grid;
    FramePool framePool;
    //Initializer initializer;
    IOProcessor ioProcessor;

    cv::Mat intrinsic_matrix;
    //PoseEstimator poseEstimator;
    //DepthEstimator depthEstimator;

    bool initOpenGLContext();

};

struct PoseEstimateFunctor {
    PoseEstimateFunctor(ceres::BiCubicInterpolator<ceres::Grid2D<uchar, 4>> &Ii, float *intrinsic_matrix, float *P, uchar *C)
            : Ii_(Ii), intrinsic_matrix_(intrinsic_matrix), P_(P), C_(C) {}

    template<typename T>
    bool operator() (const T* const Tc, const T* const Rc, T* residual) const {
        /*
         *  r = Ii(K * (R * P + T)) - C
         */

        T T_P[3] = {T(P_[0]), T(P_[1]), T(P_[2])};
        T T_C[3] = {T(C_[0]), T(C_[1]), T(C_[2])};
        T T_intrMat[9] = {
                T(intrinsic_matrix_[0]), T(intrinsic_matrix_[1]), T(intrinsic_matrix_[2]),
                T(intrinsic_matrix_[3]), T(intrinsic_matrix_[4]), T(intrinsic_matrix_[5]),
                T(intrinsic_matrix_[6]), T(intrinsic_matrix_[7]), T(intrinsic_matrix_[8])
        };

        T tmp[3];
        ceres::AngleAxisRotatePoint(Rc, T_P, tmp);
        tmp[0] += Tc[0];
        tmp[1] += Tc[1];
        tmp[2] += Tc[2];

        T tmp1[3];
        tmp1[0] = T_intrMat[0] * tmp[0] + T_intrMat[1] * tmp[1] + T_intrMat[2] * tmp[2];
        tmp1[1] = T_intrMat[3] * tmp[0] + T_intrMat[4] * tmp[1] + T_intrMat[5] * tmp[2];
        tmp1[2] = T_intrMat[6] * tmp[0] + T_intrMat[7] * tmp[1] + T_intrMat[8] * tmp[2];

        T f[4];
        Ii_.Evaluate(tmp1[1]/tmp1[2], tmp[0]/tmp[2], f);

        residual[0] = f[0] - T_C[0];
        residual[1] = f[1] - T_C[1];
        residual[2] = f[2] - T_C[2];
    }

    static ceres::CostFunction* Create(ceres::BiCubicInterpolator<ceres::Grid2D<uchar, 4>> &Ii, float *intrinsic_matrix, float *P, uchar *C) {
        return (new ceres::AutoDiffCostFunction<PoseEstimateFunctor, 3, 3, 3>(
                new PoseEstimateFunctor(Ii, intrinsic_matrix, P, C)
        ));
    }

private:
    const ceres::BiCubicInterpolator<ceres::Grid2D<uchar, 4>> &Ii_;
    const float *intrinsic_matrix_;
    const float *P_;
    const uchar *C_;
};

#endif //XFUSION_XFUSER_H
