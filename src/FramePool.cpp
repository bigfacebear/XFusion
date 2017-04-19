#include "FramePool.h"

Frame & FramePool::getLatestFrame()
{
    // TODO: �ڴ˴����� return ���
    return pool[0];
}

Frame & FramePool::getKeyFrame(Frame & Ii, cv::Mat intrinsic_matrix) {
    std::vector<Frame>::iterator iter, best;
    double maxScore = -1000;
    for (iter = pool.begin(); iter != pool.end(); iter++) {
        Frame &keyFrame = *iter;
        // TODO ???
    }
    return pool[0];
}
