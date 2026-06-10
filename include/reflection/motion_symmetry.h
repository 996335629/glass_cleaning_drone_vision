#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

class MotionSymmetryDetector {
public:
    struct SymmetryPair {
        cv::Point2f point1;
        cv::Point2f point2;
        cv::Point2f motion1;
        cv::Point2f motion2;
        float symmetry_score;
    };

    struct Config {
        double angle_tolerance = 10.0;
        double magnitude_tolerance = 0.3;
        int min_symmetric_pairs = 5;
    };

    MotionSymmetryDetector();
    explicit MotionSymmetryDetector(const Config &config);
    ~MotionSymmetryDetector();

    std::vector<SymmetryPair> detectSymmetry(const std::vector<cv::Point2f> &points,
        const std::vector<cv::Point2f> &motions);
    cv::Mat visualize(const cv::Mat &frame, const std::vector<SymmetryPair> &pairs);

private:
    Config config_;
};
