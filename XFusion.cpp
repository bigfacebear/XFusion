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
		Frame &keyFrame = framePool.getKeyFrame(newFrame);
		// Update the frame pool
		framePool.addFrame(newFrame);

		// Estimate depth map
		// cv::Mat dmap = depthEstimator.depthEstimate(newFrame, keyframe);
		cv::Mat dmap = depthEstimate(newFrame, keyFrame);

		fuseToGrid(grid, T, dmap);

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

void XFuser::fuseToGrid(Grid & grid, PoseMatx & T, cv::Mat dmap)
{
}

void XFuser::raycastingFromTv(PoseMatx & Tv, Grid & grid, cv::Size size, cv::Mat intrinsic_matrix, cv::Mat & IM, cv::Mat & DM) {
	IM = cv::Mat(size, CV_8UC3);
	DM = cv::Mat(size, CV_64FC1, cv::Scalar::all(-1));

	cv::Mat intrinsic_matrix_inv = intrinsic_matrix.inv();
	cv::Mat Tv_inv = cv::Mat(Tv.inv());

	cv::Mat M = intrinsic_matrix * cv::Mat(cv::Mat(Tv), cv::Rect(0, 0, 4, 3));
	cv::Mat A = cv::Mat(M, cv::Rect(0, 0, 3, 3));
	cv::Mat b = cv::Mat(M, cv::Rect(3, 0, 1, 3));
	cv::Mat T_Camera = -A.inv() * b;
	//cv::Mat T_Camera(cv::Mat(Tv), cv::Rect(3, 0, 1, 3));
	cv::Mat R_Camera(cv::Mat(Tv), cv::Rect(0, 0, 3, 3));
	cv::Mat R_Camera_inv = R_Camera.inv();

	std::cout << Tv << std::endl;
	std::cout << R_Camera << std::endl;
	std::cout << T_Camera << std::endl;

	for (int u = 0; u < size.width; u++) {
		for (int v = 0; v < size.height; v++) {
			cv::Mat p(cv::Vec3d(u, v, 1));

			p = intrinsic_matrix_inv * p;
			cv::Mat rayDir = R_Camera_inv * p;

			/*std::cout << p << std::endl;
			std::cout << rayDir << "\n" << std::endl;*/

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

