#include <iostream>
#include <opencv2\opencv.hpp>
#include <string>
#include "XFusion.h"

using namespace std;

int main(int argc, char **argv) {
	//XFuser fuser;
	//std::string videoName("video.avi");  // for example
	//fuser.load(videoName);
	//fuser.fuse();

	cv::FileStorage fs("intrinsics.xml", cv::FileStorage::READ);
	cv::Mat intrinsic_matrix_loaded;
	fs["camera_matrix"] >> intrinsic_matrix_loaded;

	XFuser fuser(intrinsic_matrix_loaded);

	cv::Mat Ii(cv::Size(320, 240), CV_8UC3);
	PoseMatx Tv(
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 1000,
		0, 0, 0, 1);

	Grid grid(256);

	cv::Mat IM(Ii.size(), CV_8UC3, cv::Scalar::all(0));
	cv::Mat DM(Ii.size(), CV_64FC1, cv::Scalar::all(0));
	fuser.raycastingFromTv(Tv, grid, Ii.size(), intrinsic_matrix_loaded, IM, DM);
	cv::imshow("IM", IM);
	cv::Mat DM_show(Ii.size(), CV_8UC1, cv::Scalar::all(0));
	double max = 0;
	for (int i = 0; i < Ii.rows; i++) {
		for (int j = 0; j < Ii.cols; j++) {
			if (DM.at<double>(i, j) == INFINITY) {
				continue;
			}
			if (DM.at<double>(i, j) > max) {
				max = DM.at<double>(i, j);
			}
		}
	}
	max = 2000;
	for (int i = 0; i < Ii.rows; i++) {
		for (int j = 0; j < Ii.cols; j++) {
			if (DM.at<double>(i, j) == INFINITY) {
				DM_show.at<uchar>(i, j) = 0;
				continue;
			}
			//cout << DM.at<double>(i, j) << endl;
			uchar tmp = (uchar)(DM.at<double>(i, j) * 255 / max);
			DM_show.at<uchar>(i, j) = 255 - tmp;
		}
	}
	cv::imshow("DM", DM_show);

	cv::waitKey(0);

	return 0;
}