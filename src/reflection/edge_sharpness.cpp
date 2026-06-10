#include "reflection/edge_sharpness.h"

EdgeSharpnessAnalyzer::SharpnessMetrics
EdgeSharpnessAnalyzer::computeSharpness(const cv::Mat &image, const cv::Rect &roi)
{
    SharpnessMetrics metrics = {};

    cv::Mat gray;
    if (image.channels() == 3) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    }
    else {
        gray = image.clone();
    }

    cv::Mat region = roi.empty() ? gray : gray(roi);

    metrics.laplacian_variance = laplacianVariance(region);
    metrics.gradient_magnitude = tenengradGradient(region);
    metrics.overall_sharpness = (metrics.laplacian_variance + metrics.gradient_magnitude) / 2.0;

    return metrics;
}

cv::Mat EdgeSharpnessAnalyzer::computeSharpnessMap(const cv::Mat &image, int window_size)
{
    cv::Mat gray;
    if (image.channels() == 3) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    }
    else {
        gray = image.clone();
    }

    cv::Mat laplacian;
    cv::Laplacian(gray, laplacian, CV_64F);

    cv::Mat abs_laplacian;
    laplacian = cv::abs(laplacian);
    laplacian.convertTo(abs_laplacian, CV_32F);

    cv::Mat sharpness_map;
    cv::blur(abs_laplacian, sharpness_map, cv::Size(window_size, window_size));

    return sharpness_map;
}

cv::Mat EdgeSharpnessAnalyzer::detectBlurryEdges(const cv::Mat &image, double threshold)
{
    cv::Mat sharpness_map = computeSharpnessMap(image, 31);

    cv::Mat binary;
    cv::threshold(sharpness_map, binary, threshold, 255, cv::THRESH_BINARY_INV);
    binary.convertTo(binary, CV_8U);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(15, 15));
    cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernel);

    return binary;
}

double EdgeSharpnessAnalyzer::laplacianVariance(const cv::Mat &gray)
{
    cv::Mat laplacian;
    cv::Laplacian(gray, laplacian, CV_64F);

    cv::Scalar mean, stddev;
    cv::meanStdDev(laplacian, mean, stddev);

    return stddev[0] * stddev[0];
}

double EdgeSharpnessAnalyzer::tenengradGradient(const cv::Mat &gray)
{
    cv::Mat gx, gy;
    cv::Sobel(gray, gx, CV_64F, 1, 0);
    cv::Sobel(gray, gy, CV_64F, 0, 1);

    cv::Mat magnitude;
    cv::magnitude(gx, gy, magnitude);

    return cv::mean(magnitude)[0];
}
