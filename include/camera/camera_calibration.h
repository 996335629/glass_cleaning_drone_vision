#pragma once

#include <opencv2/opencv.hpp>
#include <string>

class CameraCalibration {
public:
    struct StereoParams {
        cv::Mat left_camera_matrix;
        cv::Mat left_dist_coeffs;
        cv::Mat right_camera_matrix;
        cv::Mat right_dist_coeffs;
        cv::Mat R;
        cv::Mat T;
        cv::Mat R1, R2, P1, P2, Q;
        cv::Size image_size;
    };

    CameraCalibration();
    ~CameraCalibration();

    bool loadFromFile(const std::string &filepath);
    bool saveToFile(const std::string &filepath) const;

    const StereoParams &getParams() const { return params_; }
    bool isValid() const { return valid_; }

    void initRectifyMaps(cv::Mat &map1_left, cv::Mat &map2_left,
        cv::Mat &map1_right, cv::Mat &map2_right);

private:
    StereoParams params_;
    bool valid_ = false;
};
