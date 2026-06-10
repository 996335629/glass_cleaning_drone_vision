#pragma once

#include <opencv2/opencv.hpp>

class NoiseFilter {
public:
    struct Config {
        int gaussian_ksize = 5;
        double gaussian_sigma = 1.0;
        int bilateral_d = 9;
        double bilateral_sigma_color = 75.0;
        double bilateral_sigma_space = 75.0;
        bool use_bilateral = false;
    };

    NoiseFilter();
    explicit NoiseFilter(const Config &config);
    ~NoiseFilter();

    cv::Mat apply(const cv::Mat &input);
    void setConfig(const Config &config);

private:
    Config config_;
};
