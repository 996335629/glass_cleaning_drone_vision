#include "preprocessing/noise_filter.h"

NoiseFilter::NoiseFilter() : config_(Config()) {}
NoiseFilter::NoiseFilter(const Config &config) : config_(config) {}

NoiseFilter::~NoiseFilter() {}

cv::Mat NoiseFilter::apply(const cv::Mat &input)
{
    cv::Mat output;

    if (config_.use_bilateral) {
        cv::bilateralFilter(input, output, config_.bilateral_d,
            config_.bilateral_sigma_color, config_.bilateral_sigma_space);
    }
    else {
        cv::GaussianBlur(input, output,
            cv::Size(config_.gaussian_ksize, config_.gaussian_ksize),
            config_.gaussian_sigma);
    }

    return output;
}

void NoiseFilter::setConfig(const Config &config)
{
    config_ = config;
}
