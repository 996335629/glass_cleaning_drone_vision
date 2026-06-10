#pragma once

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <vector>

class DepthFusion {
public:
    struct Config {
        float min_depth = 0.1f;
        float max_depth = 10.0f;
        float depth_scale = 1000.0f;
    };

    DepthFusion();
    explicit DepthFusion(const Config &config);
    ~DepthFusion();

    std::vector<float> getDepthsAtPoints(const cv::Mat &depth_map,
        const std::vector<cv::Point2f> &points);
    Eigen::Vector3f flowToVelocity(const Eigen::Vector2f &flow, float depth,
        const cv::Mat &camera_matrix);

private:
    Config config_;
};
