#pragma once

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <vector>
#include <memory>

class FlowCalculator {
public:
    struct Config {
        int max_corners = 200;
        double quality_level = 0.01;
        double min_distance = 10.0;
        int block_size = 7;
        cv::Size win_size = cv::Size(21, 21);
        int max_level = 3;
        float max_flow_magnitude = 100.0f;
        float min_flow_magnitude = 0.5f;
    };

    struct FlowResult {
        std::vector<cv::Point2f> prev_points;
        std::vector<cv::Point2f> curr_points;
        std::vector<cv::Point2f> flow_vectors;
        std::vector<uchar> status;
        Eigen::Vector2f average_flow;
        Eigen::Vector2f median_flow;
        Eigen::Vector2f weighted_flow;
        float confidence;
        int valid_points_count;
    };

    FlowCalculator();
    explicit FlowCalculator(const Config &config);
    ~FlowCalculator();

    FlowResult calculate(const cv::Mat &prev_frame, const cv::Mat &curr_frame,
        const cv::Mat &reflection_mask = cv::Mat(),
        const cv::Mat &depth_map = cv::Mat());
    cv::Mat visualize(const cv::Mat &frame, const FlowResult &result);
    void setConfig(const Config &config);

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};
