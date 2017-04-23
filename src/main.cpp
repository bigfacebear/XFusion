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
            if (DM.at<double>(i, j) == INFINITY) {
                continue;
            }
            if (DM.at<double>(i, j) > max) {
                max = DM.at<double>(i, j);
            }
        }
    }
    max = 2000;
    for (int i = 0; i < FRAME_HEIGHT; i++) {
        for (int j = 0; j < FRAME_WIDTH; j++) {
            if (DM.at<double>(i, j) == INFINITY) {
                DM_show.at<uchar>(i, j) = 0;
                continue;
            }
            //cout << DM.at<double>(i, j) << endl;
            uchar tmp = (uchar)(DM.at<double>(i, j) * 255 / max);
            DM_show.at<uchar>(i, j) = 255 - tmp;
        }
    }
    return DM_show;
}

int main(int argc, char **argv) {
    //XFuser fuser;
    //std::string videoName("video.avi");  // for example
    //fuser.load(videoName);
    //fuser.fuse();

    cv::FileStorage fs("../config/intrinsics.xml", cv::FileStorage::READ);
    cv::Mat intrinsic_matrix_loaded;
    fs["camera_matrix"] >> intrinsic_matrix_loaded;
    std::cout << (intrinsic_matrix_loaded.type() == cv::DataType<double>::type) << std::endl;
    intrinsic_matrix_loaded.convertTo(intrinsic_matrix_loaded, cv::DataType<float>::type);
    std::cout << (intrinsic_matrix_loaded.type() == cv::DataType<float>::type) << std::endl;

    XFuser fuser(intrinsic_matrix_loaded);

    cv::Mat Ii(cv::Size(320, 240), CV_8UC3);
    PoseMatx Tv(
            cv::Matx44d(
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 400,
        0, 0, 0, 1)
    );

    Grid grid(256);
    float angleAxis[3] = {0, 0, 0};
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
        grid.getImageAndDepthFromViewPoint(Tv, intrinsic_matrix_loaded, IM, DM);

        cv::imshow("IM", IM);
        cv::Mat DM_show = showDM(DM);
        cv::imshow("DM", DM_show);

        if((char)cv::waitKey(33) == 27) {
            break;
        }
    }


//    double angleAxis[3] = {0, 0, 0};
//    while (true) {
//        cv::Mat Tvl = cv::Mat::eye(4, 4, CV_64FC1);
//        cv::Mat R(3, 3, CV_64FC1);
//        ceres::AngleAxisToRotationMatrix<double>(angleAxis, (double*)R.data);
//        std::cout << R << "\n" << std::endl;
//        for (int i = 0; i < 3; i++) {
//            memcpy(Tvl.data + Tvl.step[0] * i, R.data + R.step[0] * i, sizeof(double) * R.cols);
//        }
//
//        Tv = Tvl * Tv;
//        cv::Mat IM(Ii.size(), CV_8UC3, cv::Scalar::all(0));
//        cv::Mat DM(Ii.size(), CV_64FC1, cv::Scalar::all(0));
//        fuser.raycastingFromTv(Tv, grid, Ii.size(), intrinsic_matrix_loaded, IM, DM);
//        cv::imshow("IM", IM);
//        cv::Mat DM_show(Ii.size(), CV_8UC1, cv::Scalar::all(0));
//        double max = 0;
//        for (int i = 0; i < Ii.rows; i++) {
//            for (int j = 0; j < Ii.cols; j++) {
//                if (DM.at<double>(i, j) == INFINITY) {
//                    continue;
//                }
//                if (DM.at<double>(i, j) > max) {
//                    max = DM.at<double>(i, j);
//                }
//            }
//        }
//        max = 2000;
//        for (int i = 0; i < Ii.rows; i++) {
//            for (int j = 0; j < Ii.cols; j++) {
//                if (DM.at<double>(i, j) == INFINITY) {
//                    DM_show.at<uchar>(i, j) = 0;
//                    continue;
//                }
//                //cout << DM.at<double>(i, j) << endl;
//                uchar tmp = (uchar)(DM.at<double>(i, j) * 255 / max);
//                DM_show.at<uchar>(i, j) = 255 - tmp;
//            }
//        }
//        cv::imshow("DM", DM_show);
//
//
//        std::cout << Tvl << "\n" << std::endl;
//        angleAxis[1] += 0.01;
//        if((char)cv::waitKey(33) == 27) {
//            break;
//        }
//    }
//
//    cv::Mat IM(Ii.size(), CV_8UC3, cv::Scalar::all(0));
//    cv::Mat DM(Ii.size(), CV_64FC1, cv::Scalar::all(0));
//    fuser.raycastingFromTv(Tv, grid, Ii.size(), intrinsic_matrix_loaded, IM, DM);
//    cv::imshow("IM", IM);
//    cv::Mat DM_show(Ii.size(), CV_8UC1, cv::Scalar::all(0));
//    double max = 0;
//    for (int i = 0; i < Ii.rows; i++) {
//        for (int j = 0; j < Ii.cols; j++) {
//            if (DM.at<double>(i, j) == INFINITY) {
//                continue;
//            }
//            if (DM.at<double>(i, j) > max) {
//                max = DM.at<double>(i, j);
//            }
//        }
//    }
//    max = 2000;
//    for (int i = 0; i < Ii.rows; i++) {
//        for (int j = 0; j < Ii.cols; j++) {
//            if (DM.at<double>(i, j) == INFINITY) {
//                DM_show.at<uchar>(i, j) = 0;
//                continue;
//            }
//            //cout << DM.at<double>(i, j) << endl;
//            uchar tmp = (uchar)(DM.at<double>(i, j) * 255 / max);
//            DM_show.at<uchar>(i, j) = 255 - tmp;
//        }
//    }
//    cv::imshow("DM", DM_show);
//
//    cv::waitKey(0);

    return 0;
}