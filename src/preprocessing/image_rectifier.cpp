#include "preprocessing/image_rectifier.h"

ImageRectifier::ImageRectifier() {}

ImageRectifier::~ImageRectifier() {}

void ImageRectifier::initialize(const CameraCalibration &calibration)
{
    if (!calibration.isValid()) return;

    CameraCalibration &calib = const_cast<CameraCalibration &>(calibration);
    calib.initRectifyMaps(map1_left_, map2_left_, map1_right_, map2_right_);
    initialized_ = true;
}

void ImageRectifier::rectify(const cv::Mat &left_input, const cv::Mat &right_input,
    cv::Mat &left_output, cv::Mat &right_output)
{
    if (!initialized_) {
        left_output = left_input.clone();
        right_output = right_input.clone();
        return;
    }

    cv::remap(left_input, left_output, map1_left_, map2_left_, cv::INTER_LINEAR);
    cv::remap(right_input, right_output, map1_right_, map2_right_, cv::INTER_LINEAR);
}
