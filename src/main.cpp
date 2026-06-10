// src/main.cpp
#include <iostream>
#include <chrono>
#include <string>
#include <thread>
#include <opencv2/opencv.hpp>

#ifdef USE_ORBBEC_SDK
#include "camera/gemini_camera.h"
#endif

#include "estimation/displacement_estimator.h"
#include "estimation/kalman_filter.h"
#include "imu/imu_processor.h"
#include "detection/window_frame_detector.h"
#include "communication/uart_interface.h"
#include "communication/protocol.h"
#include "config/system_config_loader.h"

#ifdef SIMULATION_MODE
#include "simulator/glass_simulator.h"
#endif

// FPS计算器
class FPSCounter {
public:
    void tick()
    {
        frame_count_++;
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_time_).count();

        if (elapsed >= 1000) {
            fps_ = frame_count_ * 1000.0 / elapsed;
            frame_count_ = 0;
            last_time_ = now;
        }
    }

    double getFPS() const { return fps_; }

private:
    int frame_count_ = 0;
    double fps_ = 0.0;
    std::chrono::steady_clock::time_point last_time_ = std::chrono::steady_clock::now();
};

// 运行参数由 SystemConfig（YAML 驱动）统一管理

#ifdef SIMULATION_MODE
int runSimulation()
{
    std::cout << "=== Simulation Mode ===" << std::endl;

    int  process_width     = 320;
    int  process_height    = 240;
    bool enable_reflection = false;
    FPSCounter fps_counter;

    // 创建模拟器
    GlassSimulator::SceneConfig sim_cfg;
    sim_cfg.frame_size = cv::Size(640, 480);
    GlassSimulator simulator(sim_cfg);

    // 创建位移估计器
    DisplacementEstimator::Config est_cfg;
    est_cfg.max_features             = 40;
    est_cfg.enable_reflection_filter = enable_reflection;
    DisplacementEstimator estimator(est_cfg);

    // 卡尔曼滤波器
    KalmanFilter kf;

    cv::namedWindow("Glass Cleaning Drone Vision", cv::WINDOW_AUTOSIZE);

    bool running = true;
    cv::Point2f velocity(0, 0);

    std::cout << "\nControls:" << std::endl;
    std::cout << "  W/A/S/D - Move drone" << std::endl;
    std::cout << "  R - Reset position" << std::endl;
    std::cout << "  F - Toggle reflection filter" << std::endl;
    std::cout << "  1/2/3 - Resolution (160/320/640)" << std::endl;
    std::cout << "  ESC - Exit" << std::endl;

    while (running) {
        fps_counter.tick();

        // 更新模拟器
        simulator.updateDroneMotion(velocity, 0);

        // 生成模拟帧
        cv::Mat color, right, depth;
        simulator.generateFrame(color, right, depth);

        if (color.empty()) {
            continue;
        }

        auto process_start = std::chrono::steady_clock::now();

        // 降采样处理
        cv::Mat color_small;
        cv::resize(color, color_small,
            cv::Size(process_width, process_height));

        // 位移估计
        auto result = estimator.estimate(color_small);

        // 卡尔曼滤波
        cv::Point3f filtered = kf.update(cv::Point3f(
            static_cast<float>(result.dx),
            static_cast<float>(result.dy),
            static_cast<float>(result.dz)));

        auto process_end = std::chrono::steady_clock::now();
        double process_time = std::chrono::duration_cast<std::chrono::microseconds>(
            process_end - process_start).count() / 1000.0;

        // 缩放系数
        double scale_x = (double)color.cols / process_width;
        double scale_y = (double)color.rows / process_height;

        // 可视化
        cv::Mat display = color.clone();

        {
            // 绘制特征点
            for (size_t i = 0; i < result.features.size(); i++) {
                cv::Point2f pt(result.features[i].x * (float)scale_x,
                    result.features[i].y * (float)scale_y);
                cv::Scalar c = (i < result.inliers.size() && result.inliers[i]) ?
                    cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
                cv::circle(display, pt, 4, c, -1);
            }

            // 绘制位移箭头
            cv::Point center(display.cols / 2, display.rows / 2);
            cv::Point arrow_end(
                center.x + (int)(filtered.x * scale_x * 10),
                center.y + (int)(filtered.y * scale_y * 10)
            );
            cv::arrowedLine(display, center, arrow_end,
                cv::Scalar(0, 255, 255), 3, cv::LINE_AA);
        }

        // 显示信息
        int y = 25;
        auto putText = [&](const std::string &text, cv::Scalar c = cv::Scalar(0, 255, 0)) {
            cv::putText(display, text, cv::Point(11, y + 1),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 3);
            cv::putText(display, text, cv::Point(10, y),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, c, 2);
            y += 25;
            };

        char buf[128];
        snprintf(buf, sizeof(buf), "FPS: %.1f | Process: %.1f ms",
            fps_counter.getFPS(), process_time);
        putText(buf, cv::Scalar(255, 255, 0));

        snprintf(buf, sizeof(buf), "Points: %d/%d", result.valid_points, result.total_points);
        putText(buf);

        snprintf(buf, sizeof(buf), "Confidence: %.2f", result.confidence);
        putText(buf);

        snprintf(buf, sizeof(buf), "Displacement: (%.2f, %.2f, %.2f)",
            filtered.x, filtered.y, filtered.z);
        putText(buf);

        snprintf(buf, sizeof(buf), "Resolution: %dx%d",
            process_width, process_height);
        putText(buf);

        putText(enable_reflection ? "Reflection: ON" : "Reflection: OFF",
            enable_reflection ? cv::Scalar(0, 0, 255) : cv::Scalar(128, 128, 128));

        cv::imshow("Glass Cleaning Drone Vision", display);

        // 重置速度
        velocity = cv::Point2f(0, 0);

        // 按键处理
        int key = cv::waitKey(1);
        switch (key) {
            case 27: running = false; break;
            case 'w': case 'W': velocity.y = -5; break;
            case 's': case 'S': velocity.y = 5; break;
            case 'a': case 'A': velocity.x = -5; break;
            case 'd': case 'D': velocity.x = 5; break;
            case 'r': case 'R':
                simulator.reset();
                estimator.reset();
                kf.reset();
                std::cout << "Reset" << std::endl;
                break;
            case 'f': case 'F':
                enable_reflection = !enable_reflection;
                estimator.setReflectionFilter(enable_reflection);
                std::cout << "Reflection filter: "
                    << (enable_reflection ? "ON" : "OFF") << std::endl;
                break;
            case '1':
                process_width = 160; process_height = 120;
                estimator.reset();
                std::cout << "Resolution: 160x120" << std::endl;
                break;
            case '2':
                process_width = 320; process_height = 240;
                estimator.reset();
                std::cout << "Resolution: 320x240" << std::endl;
                break;
            case '3':
                process_width = 640; process_height = 480;
                estimator.reset();
                std::cout << "Resolution: 640x480" << std::endl;
                break;
        }
    }

    cv::destroyAllWindows();
    return 0;
}
#endif

#ifdef USE_ORBBEC_SDK
int runCamera(const SystemConfig &sys_cfg, const std::string &uart_port, int uart_baud)
{
    std::cout << "=== Camera Mode (High Performance) ===" << std::endl;

    // 从 SystemConfig 提取运行时局部变量（部分可在运行时被按键切换）
    bool enable_reflection = sys_cfg.enable_reflection_filter;
    bool vis               = sys_cfg.enable_visualization;
    bool show_depth        = false;
    int  process_width     = sys_cfg.process_width;
    int  process_height    = sys_cfg.process_height;

    FPSCounter fps_counter;

    // 列出设备
    auto devices = GeminiCamera::listDevices();
    std::cout << "Found " << devices.size() << " camera(s)" << std::endl;

    if (devices.empty()) {
        std::cerr << "No camera found!" << std::endl;
#ifdef SIMULATION_MODE
        std::cout << "Falling back to simulation..." << std::endl;
        return runSimulation();
#else
        return -1;
#endif
    }

    for (const auto &d : devices) {
        std::cout << "  " << d.name << " [" << d.serial_number << "]" << std::endl;
    }

    // 初始化相机
    std::cout << "\nInitializing Gemini camera..." << std::endl;
    GeminiCamera camera;
    GeminiCamera::Config cam_cfg;
    cam_cfg.color_width          = sys_cfg.cam_color_width;
    cam_cfg.color_height         = sys_cfg.cam_color_height;
    cam_cfg.color_fps            = sys_cfg.cam_fps;
    cam_cfg.depth_width          = sys_cfg.cam_depth_width;
    cam_cfg.depth_height         = sys_cfg.cam_depth_height;
    cam_cfg.depth_fps            = sys_cfg.cam_fps;
    cam_cfg.enable_color         = sys_cfg.cam_enable_color;
    cam_cfg.enable_depth         = sys_cfg.cam_enable_depth;
    cam_cfg.align_depth_to_color = sys_cfg.cam_align_depth;

    if (!camera.initialize(cam_cfg)) {
        std::cerr << "Failed to initialize camera: " << camera.getLastError() << std::endl;
        return -1;
    }

    if (!camera.start()) {
        std::cerr << "Failed to start camera: " << camera.getLastError() << std::endl;
        return -1;
    }

    std::cout << "Camera started successfully!" << std::endl;

    // 创建位移估计器
    DisplacementEstimator::Config est_cfg;
    est_cfg.max_features             = sys_cfg.max_features;
    est_cfg.enable_reflection_filter = enable_reflection;
    DisplacementEstimator estimator(est_cfg);

    // 卡尔曼滤波器
    KalmanFilter kf;

    // IMU 姿态解算器
    ImuProcessor imu_proc;

    // 幕墙竖框检测器（参数从 YAML 加载）
    WindowFrameDetector::Config det_cfg;
    det_cfg.canny_low      = sys_cfg.det_canny_low;
    det_cfg.canny_high     = sys_cfg.det_canny_high;
    det_cfg.hough_min_len  = sys_cfg.det_hough_min_len;
    det_cfg.hough_max_gap  = sys_cfg.det_hough_max_gap;
    det_cfg.cluster_dist   = sys_cfg.det_cluster_dist;
    det_cfg.vertical_angle = sys_cfg.det_vertical_angle;
    WindowFrameDetector frame_detector(det_cfg);

    // UART 串口
    UARTInterface uart;
    bool uart_enabled = false;
    if (!uart_port.empty()) {
        UARTInterface::Config ucfg;
        ucfg.port_name  = uart_port;
        ucfg.baud_rate  = static_cast<UARTInterface::BaudRate>(uart_baud);
        uart_enabled    = uart.open(ucfg);
        if (!uart_enabled) {
            std::cerr << "[UART] Failed to open " << uart_port << ", continuing without UART." << std::endl;
        }
    }

    // 上次有效速度（深度失效时保持上次值）
    float last_vx = 0.0f, last_vy = 0.0f, last_vz = 0.0f;

    if (vis) {
        cv::namedWindow("Drone Vision - Camera", cv::WINDOW_AUTOSIZE);
    }

    bool running = true;
    double avg_process_time = 0;
    auto last_frame_time = std::chrono::steady_clock::now();

    std::cout << "\nControls:" << std::endl;
    std::cout << "  F - Toggle reflection filter" << std::endl;
    std::cout << "  D - Toggle depth display" << std::endl;
    std::cout << "  R - Reset estimator" << std::endl;
    std::cout << "  1/2/3 - Resolution (160/320/640)" << std::endl;
    std::cout << "  ESC - Exit" << std::endl;

    while (running) {
        fps_counter.tick();

        // 计算实际帧间时间 dt
        auto now = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(now - last_frame_time).count();
        last_frame_time = now;

        // 获取相机帧
        GeminiCamera::Frame frame = camera.getFrame(100);

        if (!frame.valid || frame.color.empty()) {
            continue;
        }

        // 更新 IMU 姿态解算
        imu_proc.update(frame.imu, dt);

        auto process_start = std::chrono::steady_clock::now();

        // 降采样处理
        cv::Mat color_small;
        cv::resize(frame.color, color_small,
            cv::Size(process_width, process_height),
            0, 0, cv::INTER_LINEAR);

        // 深度图同步缩放
        cv::Mat depth_small;
        if (!frame.depth.empty()) {
            cv::resize(frame.depth, depth_small,
                cv::Size(process_width, process_height),
                0, 0, cv::INTER_NEAREST);
        }

        // 位移估计（传入深度图）
        auto result = estimator.estimate(color_small, depth_small);

        // 卡尔曼滤波（传入实际 dt）
        cv::Point3f filtered = kf.update(cv::Point3f(
            static_cast<float>(result.dx),
            static_cast<float>(result.dy),
            static_cast<float>(result.dz)), dt);

        // 幕墙竖框检测（在灰度缩放图上）
        cv::Mat gray_detect;
        cv::cvtColor(color_small, gray_detect, cv::COLOR_BGR2GRAY);
        auto bar = frame_detector.detect(gray_detect,
            result.depth_valid ? result.depth_mm : 0.0f,
            est_cfg.focal_length_px);

        // 速度换算：kf.getVelocity() 单位为像素/s → m/s
        cv::Point3f vel_px = kf.getVelocity();
        float vx_ms = last_vx, vy_ms = last_vy, vz_ms = last_vz;
        if (result.depth_valid) {
            float Z = result.depth_mm / 1000.0f;  // mm → m
            float f = est_cfg.focal_length_px;
            vx_ms = vel_px.x * Z / f;
            vy_ms = vel_px.y * Z / f;
            vz_ms = 0.0f;  // 暂留0，特征点间距估计不稳定
            last_vx = vx_ms; last_vy = vy_ms; last_vz = vz_ms;
        }

        // UART 发送
        if (uart_enabled) {
            Protocol::VelocityData vd;
            vd.vx            = vx_ms;
            vd.vy            = vy_ms;
            vd.vz            = vz_ms;
            vd.confidence    = static_cast<float>(result.confidence);
            vd.depth_valid   = result.depth_valid ? 1 : 0;
            vd.bar_count     = static_cast<uint8_t>(bar.count);
            vd.bar_offset_mm = bar.offset_valid ? bar.offset_mm : 0.0f;
            vd.timestamp_ms  = static_cast<uint32_t>(
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    now.time_since_epoch()).count());
            auto pkt = Protocol::packVelocity(vd);
            uart.send(pkt.data(), pkt.size());
        }

        auto process_end = std::chrono::steady_clock::now();
        double process_time = std::chrono::duration_cast<std::chrono::microseconds>(
            process_end - process_start).count() / 1000.0;
        avg_process_time = avg_process_time * 0.9 + process_time * 0.1;

        // 缩放系数
        double scale_x = (double)frame.color.cols / process_width;
        double scale_y = (double)frame.color.rows / process_height;

        if (!vis) {
            // 无头模式：每秒打印一次关键状态到终端
            static auto last_print = std::chrono::steady_clock::now();
            auto now_print = std::chrono::steady_clock::now();
            if (std::chrono::duration<float>(now_print - last_print).count() >= 1.0f) {
                last_print = now_print;
                char sbuf[256];
                snprintf(sbuf, sizeof(sbuf),
                    "[Headless] FPS:%.1f | Proc:%.1fms | Pts:%d | Conf:%.2f | "
                    "Vel:(%.3f,%.3f,%.3f)m/s | Depth:%s(%.0fmm) | Bar:%d(%.1fmm)",
                    fps_counter.getFPS(), avg_process_time,
                    result.valid_points,  result.confidence,
                    vx_ms, vy_ms, vz_ms,
                    result.depth_valid ? "OK" : "NO", result.depth_mm,
                    bar.count, bar.offset_valid ? bar.offset_mm : 0.0f);
                std::cout << sbuf << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        // 显示图像
        cv::Mat display = frame.color.clone();

        // 显示反射区域（红色半透明）
        if (enable_reflection && !result.reflection_mask.empty()) {
            // 将掩码缩放回原始尺寸
            cv::Mat mask_fullsize;
            cv::resize(result.reflection_mask, mask_fullsize, display.size(), 0, 0, cv::INTER_NEAREST);

            // 创建红色叠加层
            cv::Mat red_overlay = cv::Mat::zeros(display.size(), CV_8UC3);
            red_overlay.setTo(cv::Scalar(0, 0, 255));  // BGR红色

            // 将红色叠加到反射区域
            for (int y = 0; y < display.rows; y++) {
                for (int x = 0; x < display.cols; x++) {
                    if (mask_fullsize.at<uchar>(y, x) > 0) {
                        cv::Vec3b &pixel = display.at<cv::Vec3b>(y, x);
                        pixel[0] = static_cast<uchar>(pixel[0] * 0.5 + 0);       // B
                        pixel[1] = static_cast<uchar>(pixel[1] * 0.5 + 0);       // G
                        pixel[2] = static_cast<uchar>(pixel[2] * 0.5 + 127);     // R
                    }
                }
            }
        }

        {
            // 绘制特征点
            for (size_t i = 0; i < result.features.size(); i++) {
                cv::Point2f pt(result.features[i].x * (float)scale_x,
                    result.features[i].y * (float)scale_y);
                cv::Scalar c = (i < result.inliers.size() && result.inliers[i]) ?
                    cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
                cv::circle(display, pt, 4, c, -1);
            }

            // 绘制位移箭头
            cv::Point center(display.cols / 2, display.rows / 2);
            cv::Point arrow_end(
                center.x + (int)(filtered.x * scale_x * 10),
                center.y + (int)(filtered.y * scale_y * 10)
            );
            cv::arrowedLine(display, center, arrow_end,
                cv::Scalar(0, 255, 255), 3, cv::LINE_AA);
        }

        // 显示信息
        int y = 25;
        auto putText = [&](const std::string &text, cv::Scalar c = cv::Scalar(0, 255, 0)) {
            cv::putText(display, text, cv::Point(11, y + 1),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 3);
            cv::putText(display, text, cv::Point(10, y),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, c, 2);
            y += 25;
            };

        char buf[128];
        snprintf(buf, sizeof(buf), "FPS: %.1f | Process: %.1f Hz",
            fps_counter.getFPS(), 1000.0 / avg_process_time);
        putText(buf, cv::Scalar(255, 255, 0));

        snprintf(buf, sizeof(buf), "Process Time: %.2f ms", avg_process_time);
        putText(buf, cv::Scalar(255, 255, 0));

        snprintf(buf, sizeof(buf), "Points: %d/%d", result.valid_points, result.total_points);
        putText(buf);

        snprintf(buf, sizeof(buf), "Confidence: %.2f", result.confidence);
        putText(buf);

        snprintf(buf, sizeof(buf), "Pixel: (%.2f, %.2f, %.2f) px",
            filtered.x, filtered.y, filtered.z);
        putText(buf);

        // 深度信息
        if (result.depth_valid) {
            snprintf(buf, sizeof(buf), "Depth: %.0f mm (%.0f%%)",
                result.depth_mm, result.depth_valid_ratio * 100);
            putText(buf, cv::Scalar(0, 255, 0));

            snprintf(buf, sizeof(buf), "Real: (%.1f, %.1f) mm",
                result.real_dx_mm, result.real_dy_mm);
            putText(buf, cv::Scalar(0, 255, 0));

            snprintf(buf, sizeof(buf), "Drone: (%.1f, %.1f, %.1f) mm",
                result.drone_dx_mm, result.drone_dy_mm, result.drone_dz_mm);
            putText(buf, cv::Scalar(0, 200, 255));
        } else {
            snprintf(buf, sizeof(buf), "Depth: N/A (mirror?) %.0f%%",
                result.depth_valid_ratio * 100);
            putText(buf, cv::Scalar(0, 100, 255));
            putText("Real: -- (no depth)", cv::Scalar(128, 128, 128));
        }

        snprintf(buf, sizeof(buf), "Resolution: %dx%d",
            process_width, process_height);
        putText(buf);

        // 显示IMU数据
        if (frame.imu.valid) {
            snprintf(buf, sizeof(buf), "Accel: (%.2f, %.2f, %.2f) m/s2",
                frame.imu.accel_x, frame.imu.accel_y, frame.imu.accel_z);
            putText(buf, cv::Scalar(255, 200, 0));

            snprintf(buf, sizeof(buf), "Gyro:  (%.3f, %.3f, %.3f) rad/s",
                frame.imu.gyro_x, frame.imu.gyro_y, frame.imu.gyro_z);
            putText(buf, cv::Scalar(255, 200, 0));

            snprintf(buf, sizeof(buf), "IMU Temp: %.1f C", frame.imu.temperature);
            putText(buf, cv::Scalar(200, 200, 200));
        } else {
            putText("IMU: N/A", cv::Scalar(128, 128, 128));
        }

        // 显示姿态解算结果
        auto attitude = imu_proc.getAttitude();
        if (attitude.valid) {
            float roll_deg  = attitude.roll  * 180.0f / 3.14159f;
            float pitch_deg = attitude.pitch * 180.0f / 3.14159f;
            snprintf(buf, sizeof(buf), "Roll: %.1f  Pitch: %.1f deg",
                roll_deg, pitch_deg);
            putText(buf, cv::Scalar(100, 255, 255));
        }

        // 叠加幕墙竖框线段（黄色，缩放回原始分辨率）
        if (bar.detected) {
            for (const auto &l : bar.debug_lines) {
                cv::Point p1(static_cast<int>(l[0] * scale_x),
                             static_cast<int>(l[1] * scale_y));
                cv::Point p2(static_cast<int>(l[2] * scale_x),
                             static_cast<int>(l[3] * scale_y));
                cv::line(display, p1, p2, cv::Scalar(0, 220, 255), 2, cv::LINE_AA);
            }
        }

        // 竖框检测信息
        if (bar.detected) {
            char bar_buf[128];
            if (bar.offset_valid) {
                snprintf(bar_buf, sizeof(bar_buf), "Bar: %d bars, offset: %+.1f mm",
                    bar.count, bar.offset_mm);
            } else {
                snprintf(bar_buf, sizeof(bar_buf), "Bar: %d bars (no depth)", bar.count);
            }
            putText(bar_buf, cv::Scalar(0, 220, 255));
        } else {
            putText("Bar: none", cv::Scalar(128, 128, 128));
        }

        // 速度输出
        {
            char vel_buf[128];
            snprintf(vel_buf, sizeof(vel_buf), "Vel: (%.3f, %.3f, %.3f) m/s",
                vx_ms, vy_ms, vz_ms);
            putText(vel_buf, result.depth_valid ? cv::Scalar(0, 255, 200) : cv::Scalar(128, 128, 128));
        }

        // UART 状态
        if (!uart_port.empty()) {
            putText(uart_enabled
                ? ("UART: " + uart_port + " [OK]")
                : ("UART: " + uart_port + " [FAIL]"),
                uart_enabled ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255));
        }

        // 显示深度图
        if (show_depth && !frame.depth.empty()) {
            cv::Mat depth_vis;
            double min_val, max_val;
            cv::minMaxLoc(frame.depth, &min_val, &max_val);
            if (max_val > 0) {
                frame.depth.convertTo(depth_vis, CV_8U, 255.0 / max_val);
                cv::applyColorMap(depth_vis, depth_vis, cv::COLORMAP_JET);
                cv::resize(depth_vis, depth_vis, cv::Size(160, 120));
                depth_vis.copyTo(display(cv::Rect(display.cols - 170, display.rows - 130, 160, 120)));
            }
        }

        cv::imshow("Drone Vision - Camera", display);

        // 按键处理
        int key = cv::waitKey(1);
        switch (key) {
            case 27: running = false; break;
            case 'r': case 'R':
                estimator.reset();
                kf.reset();
                std::cout << "Estimator reset" << std::endl;
                break;
            case 'f': case 'F':
                enable_reflection = !enable_reflection;
                estimator.setReflectionFilter(enable_reflection);
                std::cout << "Reflection filter: "
                    << (enable_reflection ? "ON" : "OFF") << std::endl;
                break;
            case 'd': case 'D':
                show_depth = !show_depth;
                break;
            case '1':
                process_width = 160; process_height = 120;
                estimator.reset();
                std::cout << "Resolution: 160x120" << std::endl;
                break;
            case '2':
                process_width = 320; process_height = 240;
                estimator.reset();
                std::cout << "Resolution: 320x240" << std::endl;
                break;
            case '3':
                process_width = 640; process_height = 480;
                estimator.reset();
                std::cout << "Resolution: 640x480" << std::endl;
                break;
        }
    }

    camera.stop();
    cv::destroyAllWindows();
    return 0;
}
#endif

int main(int argc, char **argv)
{
    std::cout << "========================================" << std::endl;
    std::cout << "  Glass Cleaning Drone Vision System   " << std::endl;
    std::cout << "  High Performance Version             " << std::endl;
    std::cout << "========================================" << std::endl;

    // [✅] Step 1: 先扫描 --config，加载 YAML 配置
    std::string config_path = "config/system_config.yaml";
    for (int i = 1; i < argc; i++) {
        if (std::string(argv[i]) == "--config" && i + 1 < argc) {
            config_path = argv[++i];
        }
    }
    SystemConfig sys_cfg = loadSystemConfig(config_path);

    // [✅] Step 2: 解析其余命令行参数（--port/--baud 覆盖 YAML）
    std::string mode      = sys_cfg.mode;
    std::string uart_port = sys_cfg.uart_port;
    int         uart_baud = sys_cfg.uart_baudrate;

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--simulation" || arg == "-s") {
            mode = "simulation";
        }
        else if (arg == "--camera" || arg == "-c") {
            mode = "camera";
        }
        else if (arg == "--port" && i + 1 < argc) {
            uart_port = argv[++i];
        }
        else if (arg == "--baud" && i + 1 < argc) {
            uart_baud = std::stoi(argv[++i]);
        }
        else if (arg == "--config" && i + 1 < argc) {
            ++i; // 已在 Step 1 处理
        }
        else if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: " << argv[0] << " [options]" << std::endl;
            std::cout << "  --camera, -c         Camera mode (default)" << std::endl;
            std::cout << "  --simulation, -s     Simulation mode" << std::endl;
            std::cout << "  --config <path>      Config YAML (default: config/system_config.yaml)" << std::endl;
            std::cout << "  --port <name>        UART port, overrides YAML (e.g. COM3 or /dev/ttyUSB0)" << std::endl;
            std::cout << "  --baud <rate>        UART baud rate, overrides YAML (default: 115200)" << std::endl;
            std::cout << "  --help, -h           Show help" << std::endl;
            return 0;
        }
    }

    // YAML enable_uart=true 且 port 非空时，不需要 --port 也能启用
    if (sys_cfg.uart_enable && !sys_cfg.uart_port.empty() && uart_port.empty()) {
        uart_port = sys_cfg.uart_port;
    }

    std::cout << "Mode: " << mode << std::endl;
    std::cout << "Visualization: " << (sys_cfg.enable_visualization ? "ON" : "OFF (headless)") << std::endl;

#ifdef SIMULATION_MODE
    if (mode == "simulation") {
        return runSimulation();
    }
#endif

#ifdef USE_ORBBEC_SDK
    if (mode == "camera") {
        return runCamera(sys_cfg, uart_port, uart_baud);
    }
#endif

    std::cerr << "Mode not supported in this build." << std::endl;
    return -1;
}
