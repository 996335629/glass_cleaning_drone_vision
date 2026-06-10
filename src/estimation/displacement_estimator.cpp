// src/estimation/displacement_estimator.cpp
#include "estimation/displacement_estimator.h"
#include <opencv2/video/tracking.hpp>
#include <iostream>
#include <cmath>

class DisplacementEstimator::Impl {
public:
    Config config;
    cv::Mat prev_gray;
    std::vector<cv::Point2f> prev_points;
    bool initialized = false;

    void detectFeatures(const cv::Mat &gray, std::vector<cv::Point2f> &points)
    {
        points.clear();
        cv::goodFeaturesToTrack(
            gray,
            points,
            config.max_features,
            config.quality_level,
            config.min_distance,
            cv::Mat(),
            3,
            false,
            0.04
        );
    }

    // 增强版反射和透明物体检测
    cv::Mat detectReflection(const cv::Mat &color)
    {
        cv::Mat hsv, lab;
        cv::cvtColor(color, hsv, cv::COLOR_BGR2HSV);
        cv::cvtColor(color, lab, cv::COLOR_BGR2Lab);

        std::vector<cv::Mat> hsv_channels, lab_channels;
        cv::split(hsv, hsv_channels);
        cv::split(lab, lab_channels);

        cv::Mat &saturation = hsv_channels[1];
        cv::Mat &value = hsv_channels[2];
        cv::Mat &luminance = lab_channels[0];

        // === 方法1: 高亮度 + 低饱和度 = 强反射 ===
        cv::Mat bright_mask, low_sat_mask, reflection_mask1;
        cv::threshold(value, bright_mask, config.brightness_threshold, 255, cv::THRESH_BINARY);
        cv::threshold(saturation, low_sat_mask, config.saturation_threshold, 255, cv::THRESH_BINARY_INV);
        cv::bitwise_and(bright_mask, low_sat_mask, reflection_mask1);

        // === 方法2: 透明物体检测 - 边缘密集区域 ===
        cv::Mat gray, edges, reflection_mask2;
        cv::cvtColor(color, gray, cv::COLOR_BGR2GRAY);

        // Canny边缘检测
        cv::Canny(gray, edges, 50, 150);

        // 膨胀边缘，形成连通区域
        cv::Mat kernel_dilate = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(15, 15));
        cv::dilate(edges, reflection_mask2, kernel_dilate);

        // 计算局部边缘密度
        cv::Mat edge_density;
        cv::Mat kernel_blur = cv::Mat::ones(21, 21, CV_32F) / (21.0 * 21.0);
        cv::filter2D(edges, edge_density, CV_32F, kernel_blur);

        // 高边缘密度区域可能是透明物体
        cv::Mat high_edge_density;
        cv::threshold(edge_density, high_edge_density, 15, 255, cv::THRESH_BINARY);
        high_edge_density.convertTo(high_edge_density, CV_8U);

        // === 方法3: 颜色变化检测（透明物体会透过背景色）===
        cv::Mat color_variance, reflection_mask3;

        // 计算局部颜色方差
        cv::Mat color_float;
        color.convertTo(color_float, CV_32F);

        cv::Mat mean_color, mean_sq, variance;
        cv::blur(color_float, mean_color, cv::Size(11, 11));
        cv::blur(color_float.mul(color_float), mean_sq, cv::Size(11, 11));
        variance = mean_sq - mean_color.mul(mean_color);

        // 转换为灰度方差
        std::vector<cv::Mat> var_channels;
        cv::split(variance, var_channels);
        color_variance = (var_channels[0] + var_channels[1] + var_channels[2]) / 3.0;

        // 低方差 + 中等亮度 = 可能是透明区域
        cv::Mat low_variance;
        cv::threshold(color_variance, low_variance, 100, 255, cv::THRESH_BINARY_INV);
        low_variance.convertTo(low_variance, CV_8U);

        cv::Mat medium_bright;
        cv::inRange(value, 100, 220, medium_bright);

        cv::bitwise_and(low_variance, medium_bright, reflection_mask3);
        cv::bitwise_and(reflection_mask3, high_edge_density, reflection_mask3);

        // === 合并所有检测结果 ===
        cv::Mat combined_mask;
        cv::bitwise_or(reflection_mask1, reflection_mask3, combined_mask);

        // 形态学操作
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7));
        cv::morphologyEx(combined_mask, combined_mask, cv::MORPH_CLOSE, kernel);
        cv::morphologyEx(combined_mask, combined_mask, cv::MORPH_OPEN, kernel);

        // 过滤小区域
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(combined_mask.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        cv::Mat final_mask = cv::Mat::zeros(combined_mask.size(), CV_8UC1);
        for (const auto &contour : contours) {
            double area = cv::contourArea(contour);
            if (area >= config.min_region_area) {
                // 计算轮廓的矩形度和圆形度，玻璃杯通常是规则形状
                cv::Rect bbox = cv::boundingRect(contour);
                double rect_ratio = (double)bbox.width / bbox.height;

                // 接受各种形状
                cv::drawContours(final_mask, std::vector<std::vector<cv::Point>>{contour},
                    0, cv::Scalar(255), cv::FILLED);
            }
        }

        return final_mask;
    }

    // 检查点是否在反射区域内
    bool isInReflection(const cv::Point2f &pt, const cv::Mat &mask)
    {
        int x = static_cast<int>(pt.x);
        int y = static_cast<int>(pt.y);
        if (x >= 0 && x < mask.cols && y >= 0 && y < mask.rows) {
            return mask.at<uchar>(y, x) > 0;
        }
        return false;
    }
};

DisplacementEstimator::DisplacementEstimator()
    : impl_(std::make_unique<Impl>())
{
}

DisplacementEstimator::DisplacementEstimator(const Config &config)
    : impl_(std::make_unique<Impl>())
{
    impl_->config = config;
}

DisplacementEstimator::~DisplacementEstimator() = default;

DisplacementEstimator::Result DisplacementEstimator::estimate(const cv::Mat &color)
{
    cv::Mat empty_depth;
    return estimate(color, empty_depth);
}

DisplacementEstimator::Result DisplacementEstimator::estimate(
    const cv::Mat &color, const cv::Mat &depth)
{
    Result result;

    // 转换为灰度图
    cv::Mat gray;
    if (color.channels() == 3) {
        cv::cvtColor(color, gray, cv::COLOR_BGR2GRAY);
    }
    else {
        gray = color.clone();
    }

    // 反射检测
    cv::Mat reflection_mask;
    if (impl_->config.enable_reflection_filter && color.channels() == 3) {
        reflection_mask = impl_->detectReflection(color);
        result.reflection_mask = reflection_mask.clone();
    }

    // 初始化
    if (!impl_->initialized || impl_->prev_points.empty()) {
        impl_->detectFeatures(gray, impl_->prev_points);
        impl_->prev_gray = gray.clone();
        impl_->initialized = true;

        result.total_points = impl_->prev_points.size();
        result.valid_points = 0;
        result.features = impl_->prev_points;
        result.inliers.resize(impl_->prev_points.size(), true);
        return result;
    }

    // 光流跟踪
    std::vector<cv::Point2f> curr_points;
    std::vector<uchar> status;
    std::vector<float> err;

    cv::Size win_size(impl_->config.optical_flow_win_size,
        impl_->config.optical_flow_win_size);

    cv::calcOpticalFlowPyrLK(
        impl_->prev_gray, gray,
        impl_->prev_points, curr_points,
        status, err,
        win_size,
        impl_->config.optical_flow_max_level,
        cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.03)
    );

    // 计算位移
    std::vector<cv::Point2f> good_prev, good_curr;
    result.inliers.resize(status.size(), false);
    result.filtered_points = 0;

    for (size_t i = 0; i < status.size(); i++) {
        if (status[i] && err[i] < 20.0) {
            // 检查是否在反射区域
            bool in_reflection = false;
            if (!reflection_mask.empty()) {
                in_reflection = impl_->isInReflection(curr_points[i], reflection_mask);
            }

            if (in_reflection) {
                result.filtered_points++;
                result.inliers[i] = false;
            }
            else {
                good_prev.push_back(impl_->prev_points[i]);
                good_curr.push_back(curr_points[i]);
                result.inliers[i] = true;
            }
        }
    }

    result.total_points = impl_->prev_points.size();
    result.valid_points = good_prev.size();
    result.features = curr_points;

    // 计算平均位移
    if (good_prev.size() >= 4) {
        double sum_dx = 0, sum_dy = 0;
        for (size_t i = 0; i < good_prev.size(); i++) {
            sum_dx += good_curr[i].x - good_prev[i].x;
            sum_dy += good_curr[i].y - good_prev[i].y;
        }
        result.dx = sum_dx / good_prev.size();
        result.dy = sum_dy / good_prev.size();
        result.confidence = (double)good_prev.size() / impl_->prev_points.size();

        // 深度有效性检测 + 真实位移计算
        if (!depth.empty()) {
            // 统计中心ROI内有效深度像素（排除0值和超远值）
            cv::Rect roi(depth.cols / 4, depth.rows / 4,
                         depth.cols / 2, depth.rows / 2);
            cv::Mat roi_depth = depth(roi);

            int total_px = roi_depth.rows * roi_depth.cols;
            int valid_px = 0;
            double depth_sum = 0;

            for (int r = 0; r < roi_depth.rows; r++) {
                for (int c = 0; c < roi_depth.cols; c++) {
                    uint16_t d = roi_depth.at<uint16_t>(r, c);
                    if (d > 100 && d < 10000) {   // 10cm ~ 10m 有效范围
                        depth_sum += d;
                        valid_px++;
                    }
                }
            }

            result.depth_valid_ratio = (float)valid_px / total_px;
            result.depth_valid = (result.depth_valid_ratio >= impl_->config.depth_valid_threshold);

            if (result.depth_valid && valid_px > 0) {
                result.depth_mm = (float)(depth_sum / valid_px);

                // 像素位移转真实位移（mm）
                float f = impl_->config.focal_length_px;
                float Z = result.depth_mm;
                result.real_dx_mm = (float)(result.dx * Z / f);
                result.real_dy_mm = (float)(result.dy * Z / f);

                // dz：用特征点间距变化估计
                double scale_change = 0;
                int scale_count = 0;
                for (size_t i = 0; i + 1 < good_prev.size(); i++) {
                    double dist_prev = cv::norm(good_prev[i] - good_prev[i + 1]);
                    double dist_curr = cv::norm(good_curr[i] - good_curr[i + 1]);
                    if (dist_prev > 5 && dist_curr > 5) {
                        scale_change += (dist_curr - dist_prev) / dist_prev;
                        scale_count++;
                    }
                }
                float dz_mm = 0.0f;
                if (scale_count > 0) {
                    result.dz = -scale_change / scale_count * 100;
                    dz_mm = (float)result.dz;
                }

                // 坐标系变换：相机坐标系 → 无人机机体坐标系
                // 旋转矩阵 R = Ry(pitch) * Rx(roll)
                float cp = std::cos(impl_->config.camera_pitch_rad);
                float sp = std::sin(impl_->config.camera_pitch_rad);
                float cr = std::cos(impl_->config.camera_roll_rad);
                float sr = std::sin(impl_->config.camera_roll_rad);

                // Ry(pitch) * Rx(roll) 应用到 (dx, dy, dz)
                float dx_c = result.real_dx_mm;
                float dy_c = result.real_dy_mm;
                float dz_c = dz_mm;

                result.drone_dx_mm =  cp * dx_c + sp * sr * dy_c + sp * cr * dz_c;
                result.drone_dy_mm =            + cr * dy_c       - sr * dz_c;
                result.drone_dz_mm = -sp * dx_c + cp * sr * dy_c + cp * cr * dz_c;
            }
        }
    }
    else {
        result.confidence = 0;
    }

    // 更新状态
    if (result.confidence < 0.3 || good_curr.size() < 10) {
        impl_->detectFeatures(gray, impl_->prev_points);
    }
    else {
        impl_->prev_points = good_curr;
    }
    impl_->prev_gray = gray.clone();

    return result;
}

void DisplacementEstimator::setReflectionFilter(bool enable)
{
    impl_->config.enable_reflection_filter = enable;
}

bool DisplacementEstimator::isReflectionFilterEnabled() const
{
    return impl_->config.enable_reflection_filter;
}

void DisplacementEstimator::reset()
{
    impl_->initialized = false;
    impl_->prev_points.clear();
    impl_->prev_gray.release();
}
