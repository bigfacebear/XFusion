#include "FramePool.h"

Frame & FramePool::getLatestFrame()
{
	// TODO: 在此处插入 return 语句
	return Frame();
}

Frame & FramePool::getKeyFrame(Frame & Ii, cv::Mat intrinsic_matrix) {
	std::vector<Frame>::iterator iter, best;
	double maxScore = -1000;
	for (iter = pool.begin(); iter != pool.end(); iter++) {
		Frame &keyFrame = *iter;
		// TODO ???
	}
	return Frame();
}
