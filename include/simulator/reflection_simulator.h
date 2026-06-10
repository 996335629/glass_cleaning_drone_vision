#pragma once

#include <opencv2/opencv.hpp>

class ReflectionSimulator {
public:
    struct Config {
        float reflectivity = 0.6f;
        float blur_strength = 5.0f;
        cv::Point2f reflection_offset = cv::Point2f(0, 0);
    };

    ReflectionSimulator();
    explicit ReflectionSimulator(const Config &config);
    ~ReflectionSimulator();

    cv::Mat generateReflection(const cv::Mat &drone_image, const cv::Mat &background);
    void setConfig(const Config &config);

private:
    Config config_;
};
