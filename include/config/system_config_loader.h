// include/config/system_config_loader.h
// 系统 YAML 配置加载器（cv::FileStorage，无额外依赖）
// 缺失字段保留默认值，加载失败时返回全默认配置，不抛异常
#pragma once

#include <string>
#include <opencv2/core.hpp>

struct SystemConfig {
    // [system]
    std::string mode                = "camera"; // simulation 或 camera
    bool enable_visualization       = true;     // false=无头运行（X5实机）
    bool enable_reflection_filter   = false;

    // [process]
    int process_width   = 320;
    int process_height  = 240;
    int max_features    = 40;

    // [camera]
    int  cam_color_width   = 640;
    int  cam_color_height  = 480;
    int  cam_depth_width   = 640;
    int  cam_depth_height  = 480;
    int  cam_fps           = 30;
    bool cam_enable_depth  = true;
    bool cam_enable_color  = true;
    bool cam_align_depth   = true;

    // [detection] 幕墙竖框检测
    float det_canny_low      = 30.0f;
    float det_canny_high     = 100.0f;
    int   det_hough_min_len  = 60;
    int   det_hough_max_gap  = 15;
    int   det_cluster_dist   = 25;
    float det_vertical_angle = 15.0f;

    // [communication]
    bool        uart_enable   = false;
    std::string uart_port     = "";
    int         uart_baudrate = 115200;
};

// 从 YAML 文件加载配置。
// 文件不存在或格式错误时打印 Warning 并返回全默认配置，程序继续运行。
SystemConfig loadSystemConfig(const std::string &path);
