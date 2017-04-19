#pragma once

#include <opencv2/opencv.hpp>
#include <string>

class IOProcessor {
public:

    IOProcessor() = default;

    IOProcessor(const std::string &vname);

    void loadVideo(const std::string &vname);

    cv::Mat get();
private:

};