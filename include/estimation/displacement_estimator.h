// include/estimation/displacement_estimator.h
#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>

class DisplacementEstimator {
public:
    struct Config {
        int max_features = 40;
        double quality_level = 0.03;
        double min_distance = 15.0;
        bool enable_reflection_filter = false;
        int optical_flow_win_size = 15;
        int optical_flow_max_level = 2;
        // 反射检测参数
        int brightness_threshold = 180;
        int saturation_threshold = 40;
        int min_region_area = 50;
        // 深度参数
        float focal_length_px = 460.0f;     // 相机焦距（像素），Gemini336默认值
        float depth_valid_threshold = 0.3f; // 有效深度像素比例低于此值视为镜面失效
        // 相机安装姿态（相对于无人机机体坐标系）
        // 确定安装角度后填入，0则不做变换
        float camera_roll_rad  = 0.0f;      // 绕X轴倾斜（rad）
        float camera_pitch_rad = 0.0f;      // 绕Y轴倾斜（rad），向上为正
    };

    struct Result {
        double dx = 0, dy = 0, dz = 0;
        double confidence = 0;
        int total_points = 0;
        int valid_points = 0;
        int filtered_points = 0;              // 被过滤的点数
        std::vector<cv::Point2f> features;
        std::vector<bool> inliers;
        cv::Mat reflection_mask;              // 反射区域掩码
        // 深度信息
        bool depth_valid = false;             // 深度是否可用（镜面时为false）
        float depth_mm = 0.0f;               // 有效深度均值（mm）
        float depth_valid_ratio = 0.0f;      // 有效深度像素占比（0~1）
        float real_dx_mm = 0.0f;             // 真实位移X（mm，仅depth_valid时有效）
        float real_dy_mm = 0.0f;             // 真实位移Y（mm）
        // 坐标系变换后的位移（无人机机体坐标系，仅depth_valid时有效）
        float drone_dx_mm = 0.0f;
        float drone_dy_mm = 0.0f;
        float drone_dz_mm = 0.0f;
    };

    DisplacementEstimator();
    explicit DisplacementEstimator(const Config &config);
    ~DisplacementEstimator();

    Result estimate(const cv::Mat &color, const cv::Mat &depth);
    Result estimate(const cv::Mat &color);

    void setReflectionFilter(bool enable);
    bool isReflectionFilterEnabled() const;

    void reset();

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};
