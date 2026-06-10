#pragma once

#include <opencv2/opencv.hpp>

class EdgeSharpnessAnalyzer {
public:
    struct SharpnessMetrics {
        double laplacian_variance;
        double gradient_magnitude;
        double edge_density;
        double focus_measure;
        double overall_sharpness;
    };

    static SharpnessMetrics computeSharpness(const cv::Mat &image, const cv::Rect &roi = cv::Rect());
    static cv::Mat computeSharpnessMap(const cv::Mat &image, int window_size = 31);
    static cv::Mat detectBlurryEdges(const cv::Mat &image, double threshold = 50.0);

private:
    static double laplacianVariance(const cv::Mat &gray);
    static double tenengradGradient(const cv::Mat &gray);
};
