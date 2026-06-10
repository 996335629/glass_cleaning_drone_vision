#pragma once

#include <opencv2/opencv.hpp>
#include "camera/camera_calibration.h"

class ImageRectifier {
public:
    ImageRectifier();
    ~ImageRectifier();

    void initialize(const CameraCalibration &calibration);
    void rectify(const cv::Mat &left_input, const cv::Mat &right_input,
        cv::Mat &left_output, cv::Mat &right_output);

private:
    cv::Mat map1_left_, map2_left_;
    cv::Mat map1_right_, map2_right_;
    bool initialized_ = false;
};
