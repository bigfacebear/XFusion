#pragma once

#include <opencv2\opencv.hpp>
#include <vector>


class Voxel {
public:

	Voxel() :C(0, 0, 0), S(INFINITY), W(0) {}  // TODO: The initial value of S ?

	cv::Vec3b C;
	double S;
	int W;
};

class Grid {
public:
	// The container of voxels
	Voxel*** voxels;  // TODO: make a container for it

	Grid(int K);
	~Grid();

	int getK() { return K; }

	inline Voxel* getVoxelByCoordinate(double x, double y, double z);
	inline Voxel* getVoxelByCoordinate(cv::Vec3d v);
	bool getColorAndPointByRay(cv::Vec3d p, cv::Vec3d v, cv::Vec3b &C, cv::Mat &worldP);

private:
	int K;

	double min(double a, double b) { return (a > b) ? b : a; }
	double max(double a, double b) { return (a > b) ? a : b; }
};