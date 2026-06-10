#pragma once

#include "camera/gemini_camera.h"

/**
 * @brief IMU 姿态解算器
 *
 * 使用互补滤波融合加速度计（静态倾角）和陀螺仪（动态积分），
 * 输出 roll / pitch 角度，用于相机坐标系到无人机坐标系的变换。
 *
 * 互补滤波公式：
 *   angle = alpha * (angle + gyro * dt) + (1 - alpha) * accel_angle
 *
 * alpha 越接近1：信任陀螺仪（短期准、长期漂移）
 * alpha 越接近0：信任加速度计（绝对值准、噪声大）
 * 推荐值：0.95~0.98
 */
class ImuProcessor {
public:
    struct Config {
        float alpha = 0.97f;    // 互补滤波系数
    };

    struct Attitude {
        float roll  = 0.0f;    // 滚转角（rad），绕X轴，左右倾斜
        float pitch = 0.0f;    // 俯仰角（rad），绕Y轴，前后倾斜
        bool  valid = false;   // 是否已经过至少一次更新
    };

    ImuProcessor();
    explicit ImuProcessor(const Config &config);

    /**
     * @brief 输入一帧IMU数据，更新姿态估计
     * @param imu   来自 GeminiCamera::Frame::imu
     * @param dt    距上次更新的时间间隔（秒）
     */
    void update(const GeminiCamera::IMUData &imu, float dt);

    /**
     * @brief 获取当前姿态估计
     */
    Attitude getAttitude() const { return attitude_; }

    void reset();

private:
    Config   config_;
    Attitude attitude_;
    bool     first_ = true;
};
