#include "reflection/reflection_detector.h"
#include <cmath>
#include <algorithm>
#include <deque>

class ReflectionDetector::Impl {
public:
    Config config;
    std::deque<cv::Mat> mask_history;  // 历史掩码用于时序分析
    cv::Mat accumulated_mask;           // 累积掩码
    int frame_count = 0;

    Impl(const Config &cfg) : config(cfg) {}

    // 计算光流与运动方向的一致性
    float calculateFlowConsistency(const cv::Vec2f &flow, const cv::Point2f &motion)
    {
        if (motion.x == 0 && motion.y == 0) return 0.5f;

        float flow_mag = std::sqrt(flow[0] * flow[0] + flow[1] * flow[1]);
        float motion_mag = std::sqrt(motion.x * motion.x + motion.y * motion.y);

        if (flow_mag < config.motion_threshold || motion_mag < 0.1f) {
            return 0.5f;  // 运动太小，无法判断
        }

        // 计算方向相似度（点积）
        float dot = (flow[0] * motion.x + flow[1] * motion.y) / (flow_mag * motion_mag);

        // dot > 0 表示同向（反射）
        // dot < 0 表示反向（真实背景）
        return (dot + 1.0f) / 2.0f;  // 归一化到 0-1
    }

    // 基于运动方向检测反射区域
    cv::Mat detectByMotionDirection(const cv::Mat &flow, const cv::Point2f &motion)
    {
        cv::Mat reflection_prob = cv::Mat::zeros(flow.size(), CV_32F);

        for (int y = 0; y < flow.rows; y++) {
            for (int x = 0; x < flow.cols; x++) {
                cv::Vec2f f = flow.at<cv::Vec2f>(y, x);
                float consistency = calculateFlowConsistency(f, motion);

                // 一致性高（同向）= 可能是反射
                reflection_prob.at<float>(y, x) = consistency;
            }
        }

        // 平滑处理
        cv::GaussianBlur(reflection_prob, reflection_prob, cv::Size(15, 15), 5);

        // 二值化
        cv::Mat mask;
        cv::threshold(reflection_prob, mask, config.symmetry_threshold, 255, cv::THRESH_BINARY);
        mask.convertTo(mask, CV_8U);

        return mask;
    }

    // 基于光流幅度异常检测
    cv::Mat detectByFlowMagnitude(const cv::Mat &flow, const cv::Point2f &motion)
    {
        cv::Mat magnitude = cv::Mat::zeros(flow.size(), CV_32F);
        float expected_mag = std::sqrt(motion.x * motion.x + motion.y * motion.y);

        for (int y = 0; y < flow.rows; y++) {
            for (int x = 0; x < flow.cols; x++) {
                cv::Vec2f f = flow.at<cv::Vec2f>(y, x);
                float mag = std::sqrt(f[0] * f[0] + f[1] * f[1]);
                magnitude.at<float>(y, x) = mag;
            }
        }

        // 反射区域的光流幅度通常与预期不同
        cv::Mat diff;
        cv::absdiff(magnitude, cv::Scalar(expected_mag), diff);

        cv::Mat mask;
        cv::threshold(diff, mask, expected_mag * 0.5f, 255, cv::THRESH_BINARY);
        mask.convertTo(mask, CV_8U);

        return mask;
    }

    // 基于颜色/亮度检测（反射区域通常较暗或有特殊颜色）
    cv::Mat detectByAppearance(const cv::Mat &frame)
    {
        cv::Mat gray, mask;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // 检测较暗区域（可能是无人机倒影）
        cv::threshold(gray, mask, 80, 255, cv::THRESH_BINARY_INV);

        // 形态学处理
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(15, 15));
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);

        return mask;
    }

    // 时序分析：累积多帧结果
    cv::Mat temporalAnalysis(const cv::Mat &current_mask)
    {
        mask_history.push_back(current_mask.clone());

        if (mask_history.size() > static_cast<size_t>(config.temporal_window)) {
            mask_history.pop_front();
        }

        // 累积掩码
        cv::Mat result = cv::Mat::zeros(current_mask.size(), CV_32F);
        for (const auto &mask : mask_history) {
            cv::Mat temp;
            mask.convertTo(temp, CV_32F);
            result += temp;
        }

        result /= static_cast<float>(mask_history.size());

        cv::Mat final_mask;
        cv::threshold(result, final_mask, 127, 255, cv::THRESH_BINARY);
        final_mask.convertTo(final_mask, CV_8U);

        return final_mask;
    }

    // 提取连通区域
    std::vector<cv::Rect> extractRegions(const cv::Mat &mask)
    {
        std::vector<cv::Rect> regions;
        std::vector<std::vector<cv::Point>> contours;

        cv::findContours(mask.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (const auto &contour : contours) {
            double area = cv::contourArea(contour);
            if (area >= config.min_reflection_area) {
                regions.push_back(cv::boundingRect(contour));
            }
        }

        return regions;
    }
};

ReflectionDetector::ReflectionDetector()
    : impl_(std::make_unique<Impl>(Config()))
{
}

ReflectionDetector::ReflectionDetector(const Config &config)
    : impl_(std::make_unique<Impl>(config))
{
}

ReflectionDetector::~ReflectionDetector() = default;

ReflectionDetector::DetectionResult ReflectionDetector::detect(
    const cv::Mat &frame,
    const cv::Mat &prev_frame,
    const cv::Mat &flow,
    const cv::Point2f &estimated_motion
)
{
    DetectionResult result;
    impl_->frame_count++;

    // 方法1：基于运动方向检测
    cv::Mat motion_mask = impl_->detectByMotionDirection(flow, estimated_motion);

    // 方法2：基于外观检测（暗区域）
    cv::Mat appearance_mask = impl_->detectByAppearance(frame);

    // 综合两种方法
    cv::Mat combined_mask;
    cv::bitwise_or(motion_mask, appearance_mask, combined_mask);

    // 时序分析平滑结果
    cv::Mat temporal_mask = impl_->temporalAnalysis(combined_mask);

    // 形态学处理
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(11, 11));
    cv::morphologyEx(temporal_mask, temporal_mask, cv::MORPH_CLOSE, kernel);
    cv::dilate(temporal_mask, temporal_mask, kernel);  // 扩大反射区域以确保完全覆盖

    result.reflection_mask = temporal_mask;
    result.reflection_regions = impl_->extractRegions(temporal_mask);

    // 计算反射区域占比
    int total_pixels = temporal_mask.rows * temporal_mask.cols;
    int reflection_pixels = cv::countNonZero(temporal_mask);
    result.reflection_ratio = static_cast<float>(reflection_pixels) / total_pixels;

    // 计算置信度
    result.confidence = std::min(1.0f, impl_->frame_count / 10.0f) *
        (1.0f - std::abs(result.reflection_ratio - 0.1f));

    return result;
}

std::vector<int> ReflectionDetector::filterPoints(
    const std::vector<cv::Point2f> &points,
    const cv::Mat &mask
)
{
    std::vector<int> valid_indices;

    for (size_t i = 0; i < points.size(); i++) {
        int x = static_cast<int>(points[i].x);
        int y = static_cast<int>(points[i].y);

        if (x >= 0 && x < mask.cols && y >= 0 && y < mask.rows) {
            // 如果该点不在反射区域内，则保留
            if (mask.at<uchar>(y, x) == 0) {
                valid_indices.push_back(static_cast<int>(i));
            }
        }
    }

    return valid_indices;
}

void ReflectionDetector::visualize(cv::Mat &display, const DetectionResult &result)
{
    // 绘制反射区域（红色半透明覆盖）
    cv::Mat overlay = display.clone();

    for (const auto &region : result.reflection_regions) {
        cv::rectangle(overlay, region, cv::Scalar(0, 0, 255), -1);
    }

    cv::addWeighted(overlay, 0.3, display, 0.7, 0, display);

    // 绘制反射区域边框
    for (const auto &region : result.reflection_regions) {
        cv::rectangle(display, region, cv::Scalar(0, 0, 255), 2);
    }

    // 显示反射比例
    std::string text = "Reflection: " + std::to_string(int(result.reflection_ratio * 100)) + "%";
    cv::putText(display, text, cv::Point(10, display.rows - 20),
        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
}

void ReflectionDetector::reset()
{
    impl_->mask_history.clear();
    impl_->frame_count = 0;
}
