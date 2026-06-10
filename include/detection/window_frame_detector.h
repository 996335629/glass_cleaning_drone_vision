// include/detection/window_frame_detector.h
// 幕墙竖向铝合金分格条检测
// 输出：检测到的竖条数量 + 距图像中心最近竖条的横向偏移（mm）
#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

class WindowFrameDetector {
public:
    struct Config {
        float canny_low      = 30.0f;  // Canny低阈值（金属框对比度高）
        float canny_high     = 100.0f; // Canny高阈值
        int   hough_min_len  = 60;     // 最小线段长度（px）
        int   hough_max_gap  = 15;     // 线段最大间隔（px）
        int   cluster_dist   = 25;     // 同方向线段聚类距离（px）
        float vertical_angle = 15.0f; // 与竖直方向偏差不超过此值视为竖线（度）
    };

    struct Result {
        bool  detected     = false; // 是否检测到至少一根竖条
        int   count        = 0;     // 检测到的竖条数量
        std::vector<float> bar_x;   // 各竖条图像x坐标（像素，升序）
        float nearest_x    = 0;     // 距图像中心最近竖条的x坐标
        float offset_px    = 0;     // 图像中心到最近竖条的偏移（px，正=竖条在右）
        float offset_mm    = 0;     // 换算后偏移（mm），offset_valid为true时有效
        bool  offset_valid = false; // depth_mm > 0 且 detected 时为 true
        std::vector<cv::Vec4i> debug_lines; // 所有通过过滤的原始线段（用于可视化）
    };

    WindowFrameDetector();
    explicit WindowFrameDetector(const Config &config);

    // gray：灰度图（处理分辨率）
    // depth_mm：有效深度均值（mm），0表示深度不可用
    // focal_length_px：相机焦距（像素）
    Result detect(const cv::Mat &gray,
                  float depth_mm        = 0,
                  float focal_length_px = 460.0f);

private:
    Config config_;
};
