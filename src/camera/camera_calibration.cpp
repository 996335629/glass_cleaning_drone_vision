#include "camera/camera_calibration.h"
#include <iostream>

CameraCalibration::CameraCalibration() {}

CameraCalibration::~CameraCalibration() {}

bool CameraCalibration::loadFromFile(const std::string &filepath)
{
    try {
        cv::FileStorage fs(filepath, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            std::cerr << "[CameraCalibration] Cannot open: " << filepath << std::endl;
            return false;
        }

        fs["left_camera_matrix"] >> params_.left_camera_matrix;
        fs["left_dist_coeffs"] >> params_.left_dist_coeffs;
        fs["right_camera_matrix"] >> params_.right_camera_matrix;
        fs["right_dist_coeffs"] >> params_.right_dist_coeffs;
        fs["R"] >> params_.R;
        fs["T"] >> params_.T;
        fs["image_width"] >> params_.image_size.width;
        fs["image_height"] >> params_.image_size.height;

        fs.release();
        valid_ = true;
        std::cout << "[CameraCalibration] Loaded: " << filepath << std::endl;
        return true;
    }
    catch (const std::exception &e) {
        std::cerr << "[CameraCalibration] Error: " << e.what() << std::endl;
        return false;
    }
}

bool CameraCalibration::saveToFile(const std::string &filepath) const
{
    try {
        cv::FileStorage fs(filepath, cv::FileStorage::WRITE);
        if (!fs.isOpened()) return false;

        fs << "left_camera_matrix" << params_.left_camera_matrix;
        fs << "left_dist_coeffs" << params_.left_dist_coeffs;
        fs << "right_camera_matrix" << params_.right_camera_matrix;
        fs << "right_dist_coeffs" << params_.right_dist_coeffs;
        fs << "R" << params_.R;
        fs << "T" << params_.T;
        fs << "image_width" << params_.image_size.width;
        fs << "image_height" << params_.image_size.height;

        fs.release();
        return true;
    }
    catch (...) {
        return false;
    }
}

void CameraCalibration::initRectifyMaps(cv::Mat &map1_left, cv::Mat &map2_left,
    cv::Mat &map1_right, cv::Mat &map2_right)
{
    if (!valid_) return;

    cv::stereoRectify(params_.left_camera_matrix, params_.left_dist_coeffs,
        params_.right_camera_matrix, params_.right_dist_coeffs,
        params_.image_size, params_.R, params_.T,
        params_.R1, params_.R2, params_.P1, params_.P2, params_.Q);

    cv::initUndistortRectifyMap(params_.left_camera_matrix, params_.left_dist_coeffs,
        params_.R1, params_.P1, params_.image_size,
        CV_32FC1, map1_left, map2_left);

    cv::initUndistortRectifyMap(params_.right_camera_matrix, params_.right_dist_coeffs,
        params_.R2, params_.P2, params_.image_size,
        CV_32FC1, map1_right, map2_right);
}
