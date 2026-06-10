#include "optical_flow/feature_detector.h"

FeatureDetector::FeatureDetector() : config_(Config()) {}
FeatureDetector::FeatureDetector(const Config &config) : config_(config) {}

FeatureDetector::~FeatureDetector() {}

std::vector<cv::Point2f> FeatureDetector::detect(const cv::Mat &image, const cv::Mat &mask)
{
    std::vector<cv::Point2f> corners;

    cv::Mat gray;
    if (image.channels() == 3) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    }
    else {
        gray = image;
    }

    cv::goodFeaturesToTrack(gray, corners, config_.max_corners,
        config_.quality_level, config_.min_distance,
        mask, config_.block_size,
        config_.use_harris, config_.harris_k);

    return corners;
}

void FeatureDetector::setConfig(const Config &config)
{
    config_ = config;
}
