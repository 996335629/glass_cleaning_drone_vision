// src/detection/window_frame_detector.cpp
// 幕墙竖向铝合金分格条检测实现
#include "detection/window_frame_detector.h"
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

WindowFrameDetector::WindowFrameDetector() : config_(Config()) {}

WindowFrameDetector::WindowFrameDetector(const Config &config)
    : config_(config)
{
}

WindowFrameDetector::Result WindowFrameDetector::detect(
    const cv::Mat &gray,
    float depth_mm,
    float focal_length_px)
{
    Result result;

    if (gray.empty()) {
        return result;
    }

    // [✅] Step 1: Canny 边缘检测
    cv::Mat edges;
    cv::Canny(gray, edges, config_.canny_low, config_.canny_high);

    // [✅] Step 2: HoughLinesP 检测线段
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(
        edges, lines,
        1,                          // rho 分辨率（像素）
        CV_PI / 180.0,              // theta 分辨率（弧度）
        40,                         // 投票阈值
        config_.hough_min_len,
        config_.hough_max_gap
    );

    if (lines.empty()) {
        return result;
    }

    // [✅] Step 3: 过滤近似竖线（|angle| < vertical_angle°）
    const float angle_thresh_rad = config_.vertical_angle * (float)M_PI / 180.0f;
    std::vector<cv::Vec4i> vert_lines;
    for (const auto &l : lines) {
        float dx = static_cast<float>(l[2] - l[0]);
        float dy = static_cast<float>(l[3] - l[1]);
        // 竖线：dy >> dx，angle是与水平轴的夹角，竖线接近90°
        float angle = std::abs(std::atan2(std::abs(dx), std::abs(dy))); // 与竖直方向的偏差
        if (angle < angle_thresh_rad) {
            vert_lines.push_back(l);
            result.debug_lines.push_back(l);
        }
    }

    if (vert_lines.empty()) {
        return result;
    }

    // [✅] Step 4: 按 x 坐标聚类，合并近似同一根竖条的线段
    // 计算每条线段的中心 x
    std::vector<float> line_cx;
    for (const auto &l : vert_lines) {
        line_cx.push_back(static_cast<float>(l[0] + l[2]) / 2.0f);
    }

    // 排序
    std::sort(line_cx.begin(), line_cx.end());

    // 聚类：贪心合并
    std::vector<float> clusters;
    float cluster_sum = line_cx[0];
    int cluster_cnt = 1;

    for (size_t i = 1; i < line_cx.size(); i++) {
        if (line_cx[i] - line_cx[i - 1] < config_.cluster_dist) {
            cluster_sum += line_cx[i];
            cluster_cnt++;
        } else {
            clusters.push_back(cluster_sum / cluster_cnt);
            cluster_sum = line_cx[i];
            cluster_cnt = 1;
        }
    }
    clusters.push_back(cluster_sum / cluster_cnt);

    result.bar_x  = clusters;
    result.count  = static_cast<int>(clusters.size());
    result.detected = true;

    // [✅] Step 5: 找距图像中心最近的竖条
    float cx = static_cast<float>(gray.cols) / 2.0f;
    float min_dist = std::abs(clusters[0] - cx);
    float nearest  = clusters[0];

    for (float x : clusters) {
        float d = std::abs(x - cx);
        if (d < min_dist) {
            min_dist = d;
            nearest  = x;
        }
    }

    result.nearest_x = nearest;
    result.offset_px = cx - nearest; // 正=竖条在图像右侧，无人机需向左修正

    // [✅] Step 6: 换算为 mm（需要有效深度）
    if (depth_mm > 0 && focal_length_px > 0) {
        result.offset_mm    = result.offset_px * depth_mm / focal_length_px;
        result.offset_valid = true;
    }

    return result;
}
