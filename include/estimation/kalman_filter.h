// include/estimation/kalman_filter.h
#pragma once

#include <opencv2/opencv.hpp>

class KalmanFilter {
public:
    KalmanFilter();
    ~KalmanFilter();


    void init();

    // 更新并返回滤波后的位移，dt为距上一帧的时间间隔（秒）
    cv::Point3f update(const cv::Point3f &measurement, float dt = 0.033f);

    // 获取当前位置
    cv::Point3f getPosition() const;

    // 获取当前速度
    cv::Point3f getVelocity() const;

    // 重置
    void reset();

private:
    cv::KalmanFilter kf_;
    bool initialized_;
    cv::Point3f position_;
    cv::Point3f velocity_;
};
