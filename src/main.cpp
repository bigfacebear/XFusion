#include <iostream>
#include <opencv2/opencv.hpp>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "XFuser.h"

using namespace std;

cv::Mat showDM(cv::Mat DM) {
    cv::Mat DM_show(IMG_SIZE, CV_8UC1, cv::Scalar::all(0));
    double max = 0;
    for (int i = 0; i < FRAME_HEIGHT; i++) {
        for (int j = 0; j < FRAME_WIDTH; j++) {
            if (DM.at<float>(i, j) == 2000) {
                continue;
            }
            if (DM.at<float>(i, j) > max) {
                max = DM.at<float>(i, j);
            }
        }
    }
    max = 1999;
    for (int i = 0; i < FRAME_HEIGHT; i++) {
        for (int j = 0; j < FRAME_WIDTH; j++) {
            if (DM.at<float>(i, j) == 2000) {
                DM_show.at<uchar>(i, j) = 0;
                continue;
            }
            //cout << DM.at<double>(i, j) << endl;
            uchar tmp = (uchar)(DM.at<float>(i, j) * 255 / max);
            DM_show.at<uchar>(i, j) = 255 - tmp;
        }
    }
    return DM_show;
}

int main(int argc, char **argv) {
    cv::FileStorage fs("../config/intrinsics.xml", cv::FileStorage::READ);
    cv::Mat intrinsic_matrix_loaded;
    fs["camera_matrix"] >> intrinsic_matrix_loaded;
    intrinsic_matrix_loaded.convertTo(intrinsic_matrix_loaded, cv::DataType<float>::type);

    XFuser fuser(intrinsic_matrix_loaded);

    cv::Mat Ii(cv::Size(320, 240), CV_8UC3);
    PoseMatx Tv(
            cv::Matx44f(
                    1, 0, 0, 0,
                    0, 1, 0, 0,
                    0, 0, 1, 500,
                    0, 0, 0, 1)
    );

    Grid grid(256);

    float angleAxis[3] = {0, 0.1, 0};
    Shader shader("../shader/vShader.vs", "../shader/raycastShader.frag");
    while(true) {
        cv::Mat Tvl = cv::Mat::eye(4, 4, cv::DataType<float>::type);
        cv::Mat R(3, 3, cv::DataType<float>::type);
        ceres::AngleAxisToRotationMatrix<float>(angleAxis, (float*)R.data);
        for (int i = 0; i < 3; i++) {
            memcpy(Tvl.data + Tvl.step[0] * i, R.data + R.step[0] * i, sizeof(float) * R.cols);
        }

        Tv = Tvl * Tv;
        cv::Mat IM(Ii.size(), CV_8UC3, cv::Scalar::all(0));
        cv::Mat DM(Ii.size(), CV_32FC1, cv::Scalar::all(0));
        cv::Mat WM(Ii.size(), CV_32FC1, cv::Scalar::all(0));
        grid.getImageAndDepthFromViewPoint(Tv, intrinsic_matrix_loaded, IM, DM, WM);

        cv::imshow("IM", IM);
        cv::Mat DM_show = showDM(DM);
        cv::imshow("DM", DM_show);

        if((char)cv::waitKey(33) == 27) {
            break;
        }
    }
    return 0;
}