#pragma once

#include <opencv2\opencv.hpp>
#include <vector>

typedef cv::Matx44d PoseMatx;

class Frame {
public:

	cv::Mat image;  // The image of the frame
	PoseMatx T;  // The pose of the camera

	Frame() = default; // FOR DEBUG
	Frame(cv::Mat img) : image(img), T(PoseMatx::zeros()) {}
	Frame(cv::Mat img, PoseMatx T) : image(img), T(T) {}
};

class FramePool {
public:
	FramePool(int capacity = 40) :capacity(capacity) {};

	int capacity;

	Frame& getLatestFrame();  // Get the latest frame for pose estimation

	Frame& getKeyFrame(Frame &Ii);  // Find a key frame for an incoming new frame

	void addFrame(cv::Mat img) {
		pool.push_back(Frame(img));
	}
	void addFrame(Frame &frame) {
		pool.push_back(frame);
	}

private:
	std::vector<Frame> pool;
};