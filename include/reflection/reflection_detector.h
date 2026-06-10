#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>

/**
 * @brief 反射区域检测器
 *
 * 通过多种方法检测玻璃反射区域：
 * 1. 运动对称性分析 - 倒影与真实背景运动方向相反
 * 2. 光流一致性检测 - 倒影区域光流与整体不一致
 * 3. 时序分析 - 持续跟踪可疑区域
 */
class ReflectionDetector {
public:
    struct Config {
        float motion_threshold = 2.0f;       // 最小运动阈值
        float symmetry_threshold = 0.7f;     // 对称性判断阈值
        float consistency_threshold = 0.8f;  // 一致性阈值
        int min_reflection_area = 500;       // 最小反射区域面积
        int temporal_window = 5;             // 时序分析窗口
        bool enable_visualization = true;    // 是否可视化
    };

    struct DetectionResult {
        cv::Mat reflection_mask;                    // 反射区域掩码
        std::vector<cv::Rect> reflection_regions;   // 反射区域边界框
        std::vector<cv::Point2f> valid_points;      // 有效特征点（非反射区域）
        std::vector<cv::Point2f> reflection_points; // 反射区域的特征点
        float reflection_ratio;                     // 反射区域占比
        float confidence;                           // 检测置信度
    };

    ReflectionDetector();
    explicit ReflectionDetector(const Config &config);
    ~ReflectionDetector();

    /**
     * @brief 检测反射区域
     * @param frame 当前帧
     * @param prev_frame 上一帧
     * @param flow 光流场
     * @param estimated_motion 估计的相机运动方向
     * @return 检测结果
     */
    DetectionResult detect(
        const cv::Mat &frame,
        const cv::Mat &prev_frame,
        const cv::Mat &flow,
        const cv::Point2f &estimated_motion
    );

    /**
     * @brief 过滤特征点，移除反射区域的点
     * @param points 输入特征点
     * @param mask 反射掩码
     * @return 过滤后的有效点索引
     */
    std::vector<int> filterPoints(
        const std::vector<cv::Point2f> &points,
        const cv::Mat &mask
    );

    /**
     * @brief 可视化检测结果
     */
    void visualize(cv::Mat &display, const DetectionResult &result);

    /**
     * @brief 重置检测器状态
     */
    void reset();

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};
