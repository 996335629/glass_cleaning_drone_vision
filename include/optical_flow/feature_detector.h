#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

class FeatureDetector {
public:
    struct Config {
        int max_corners = 200;
        double quality_level = 0.01;
        double min_distance = 10.0;
        int block_size = 7;
        bool use_harris = false;
        double harris_k = 0.04;
    };

    FeatureDetector();
    explicit FeatureDetector(const Config &config);
    ~FeatureDetector();

    std::vector<cv::Point2f> detect(const cv::Mat &image, const cv::Mat &mask = cv::Mat());
    void setConfig(const Config &config);

private:
    Config config_;
};
