// src/config/system_config_loader.cpp
// YAML 配置加载实现，使用 cv::FileStorage（OpenCV 内置，无额外依赖）
#include "config/system_config_loader.h"
#include <iostream>

SystemConfig loadSystemConfig(const std::string &path)
{
    SystemConfig cfg;

    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "[Config] Warning: cannot open '" << path
                  << "', using default values." << std::endl;
        return cfg;
    }

    // [✅] system
    // 注意：OpenCV FileStorage YAML 不支持 true/false，bool 字段用 int 中转
    cv::FileNode sys = fs["system"];
    if (!sys.empty()) {
        if (!sys["mode"].empty()) sys["mode"] >> cfg.mode;
        if (!sys["enable_visualization"].empty()) {
            int v = 1; sys["enable_visualization"] >> v;
            cfg.enable_visualization = (v != 0);
        }
        if (!sys["enable_reflection_filter"].empty()) {
            int v = 0; sys["enable_reflection_filter"] >> v;
            cfg.enable_reflection_filter = (v != 0);
        }
    }

    // [✅] process
    cv::FileNode proc = fs["process"];
    if (!proc.empty()) {
        if (!proc["width"].empty())        proc["width"]        >> cfg.process_width;
        if (!proc["height"].empty())       proc["height"]       >> cfg.process_height;
        if (!proc["max_features"].empty()) proc["max_features"] >> cfg.max_features;
    }

    // [✅] camera
    cv::FileNode cam = fs["camera"];
    if (!cam.empty()) {
        if (!cam["color_width"].empty())          cam["color_width"]          >> cfg.cam_color_width;
        if (!cam["color_height"].empty())         cam["color_height"]         >> cfg.cam_color_height;
        if (!cam["depth_width"].empty())          cam["depth_width"]          >> cfg.cam_depth_width;
        if (!cam["depth_height"].empty())         cam["depth_height"]         >> cfg.cam_depth_height;
        if (!cam["fps"].empty())                  cam["fps"]                  >> cfg.cam_fps;
        if (!cam["enable_depth"].empty()) {
            int v = 1; cam["enable_depth"] >> v; cfg.cam_enable_depth = (v != 0);
        }
        if (!cam["enable_color"].empty()) {
            int v = 1; cam["enable_color"] >> v; cfg.cam_enable_color = (v != 0);
        }
        if (!cam["align_depth_to_color"].empty()) {
            int v = 1; cam["align_depth_to_color"] >> v; cfg.cam_align_depth = (v != 0);
        }
    }

    // [✅] detection (幕墙竖框)
    cv::FileNode det = fs["detection"];
    if (!det.empty()) {
        if (!det["canny_low"].empty())      det["canny_low"]      >> cfg.det_canny_low;
        if (!det["canny_high"].empty())     det["canny_high"]     >> cfg.det_canny_high;
        if (!det["hough_min_len"].empty())  det["hough_min_len"]  >> cfg.det_hough_min_len;
        if (!det["hough_max_gap"].empty())  det["hough_max_gap"]  >> cfg.det_hough_max_gap;
        if (!det["cluster_dist"].empty())   det["cluster_dist"]   >> cfg.det_cluster_dist;
        if (!det["vertical_angle"].empty()) det["vertical_angle"] >> cfg.det_vertical_angle;
    }

    // [✅] communication
    cv::FileNode comm = fs["communication"];
    if (!comm.empty()) {
        if (!comm["enable_uart"].empty()) {
            int v = 0; comm["enable_uart"] >> v; cfg.uart_enable = (v != 0);
        }
        if (!comm["port"].empty())        comm["port"]        >> cfg.uart_port;
        if (!comm["baudrate"].empty())    comm["baudrate"]    >> cfg.uart_baudrate;
    }

    fs.release();
    std::cout << "[Config] Loaded: " << path << std::endl;
    return cfg;
}
