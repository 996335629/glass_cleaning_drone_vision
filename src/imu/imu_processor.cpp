#include "imu/imu_processor.h"
#include <cmath>
#include <algorithm>

ImuProcessor::ImuProcessor() : config_(Config()) {}

ImuProcessor::ImuProcessor(const Config &config)
    : config_(config)
{
}

void ImuProcessor::update(const GeminiCamera::IMUData &imu, float dt)
{
    if (!imu.valid) return;

    dt = std::max(0.001f, std::min(dt, 0.5f));

    // 加速度计静态倾角（弧度）
    // roll：绕X轴，由 ay/az 决定
    // pitch：绕Y轴，由 ax/sqrt(ay²+az²) 决定
    float accel_roll  = std::atan2(imu.accel_y, imu.accel_z);
    float accel_pitch = std::atan2(-imu.accel_x,
        std::sqrt(imu.accel_y * imu.accel_y + imu.accel_z * imu.accel_z));

    if (first_) {
        // 第一帧直接用加速度计初始化，避免从0开始大幅跳变
        attitude_.roll  = accel_roll;
        attitude_.pitch = accel_pitch;
        first_ = false;
    } else {
        // 互补滤波：陀螺仪积分 + 加速度计修正
        float gyro_roll  = attitude_.roll  + imu.gyro_x * dt;
        float gyro_pitch = attitude_.pitch + imu.gyro_y * dt;

        attitude_.roll  = config_.alpha * gyro_roll  + (1.0f - config_.alpha) * accel_roll;
        attitude_.pitch = config_.alpha * gyro_pitch + (1.0f - config_.alpha) * accel_pitch;
    }

    attitude_.valid = true;
}

void ImuProcessor::reset()
{
    attitude_ = Attitude{};
    first_ = true;
}
