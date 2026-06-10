#pragma once

#include <opencv2/opencv.hpp>
#include <memory>
#include <string>
#include <vector>

/**
 * @brief Orbbec Gemini 相机接口
 */
class GeminiCamera {
public:
    struct Config {
        int color_width = 640;
        int color_height = 480;
        int color_fps = 30;
        int depth_width = 640;
        int depth_height = 480;
        int depth_fps = 30;
        bool enable_depth = true;
        bool enable_color = true;
        bool align_depth_to_color = true;
        bool enable_imu = true;     // 启用IMU（加速度计+陀螺仪）
        std::string serial_number = "";
    };

    struct IMUData {
        float accel_x = 0, accel_y = 0, accel_z = 0;  // 加速度 (m/s²)
        float gyro_x = 0, gyro_y = 0, gyro_z = 0;     // 角速度 (rad/s)
        float temperature = 0;                          // IMU温度 (℃)
        uint64_t timestamp = 0;                         // 时间戳 (ms)
        bool valid = false;
    };

    struct Frame {
        cv::Mat color;          // BGR图像
        cv::Mat depth;          // 深度图 (16UC1, 单位mm)
        uint64_t timestamp;     // 时间戳
        bool valid;             // 是否有效
        IMUData imu;            // 最新IMU数据
    };

    struct CameraInfo {
        std::string name;
        std::string serial_number;
        int pid;
        int vid;
    };

    GeminiCamera();
    ~GeminiCamera();

    // 禁用拷贝
    GeminiCamera(const GeminiCamera &) = delete;
    GeminiCamera &operator=(const GeminiCamera &) = delete;

    /**
     * @brief 初始化相机
     */
    bool initialize();
    bool initialize(const Config &config);

    /**
     * @brief 启动相机流
     */
    bool start();

    /**
     * @brief 停止相机流
     */
    void stop();

    /**
     * @brief 获取一帧数据
     */
    Frame getFrame(int timeout_ms = 1000);

    /**
     * @brief 检查相机是否正在运行
     */
    bool isRunning() const;

    /**
     * @brief 获取相机信息
     */
    CameraInfo getCameraInfo() const;

    /**
     * @brief 列出所有可用相机
     */
    static std::vector<CameraInfo> listDevices();

    /**
     * @brief 获取最后的错误信息
     */
    std::string getLastError() const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};
