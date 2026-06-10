// src/estimation/kalman_filter.cpp
#include "estimation/kalman_filter.h"
#include <algorithm>

KalmanFilter::KalmanFilter()
    : initialized_(false)
    , position_(0, 0, 0)
    , velocity_(0, 0, 0)
{
    init();
}

KalmanFilter::~KalmanFilter() = default;

void KalmanFilter::init()
{
    // 状态: [x, y, z, vx, vy, vz] - 6维状态
    // 观测: [x, y, z] - 3维观测
    kf_ = cv::KalmanFilter(6, 3, 0);

    // 状态转移矩阵 F
    // [1, 0, 0, dt, 0,  0 ]
    // [0, 1, 0, 0,  dt, 0 ]
    // [0, 0, 1, 0,  0,  dt]
    // [0, 0, 0, 1,  0,  0 ]
    // [0, 0, 0, 0,  1,  0 ]
    // [0, 0, 0, 0,  0,  1 ]
    float dt = 0.033f;  // 约30fps
    kf_.transitionMatrix = (cv::Mat_<float>(6, 6) <<
        1, 0, 0, dt, 0, 0,
        0, 1, 0, 0, dt, 0,
        0, 0, 1, 0, 0, dt,
        0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1);

    // 观测矩阵 H
    kf_.measurementMatrix = (cv::Mat_<float>(3, 6) <<
        1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0);

    // 过程噪声协方差 Q
    cv::setIdentity(kf_.processNoiseCov, cv::Scalar::all(0.1));

    // 观测噪声协方差 R
    cv::setIdentity(kf_.measurementNoiseCov, cv::Scalar::all(1.0));

    // 后验误差协方差 P
    cv::setIdentity(kf_.errorCovPost, cv::Scalar::all(10));

    // 初始状态
    kf_.statePost = cv::Mat::zeros(6, 1, CV_32F);

    initialized_ = true;
    position_ = cv::Point3f(0, 0, 0);
    velocity_ = cv::Point3f(0, 0, 0);
}

cv::Point3f KalmanFilter::update(const cv::Point3f &measurement, float dt)
{
    if (!initialized_) {
        init();
    }

    // 用实际 dt 更新状态转移矩阵中的速度分量
    dt = std::max(0.001f, std::min(dt, 0.5f));  // 限制在合理范围
    kf_.transitionMatrix.at<float>(0, 3) = dt;
    kf_.transitionMatrix.at<float>(1, 4) = dt;
    kf_.transitionMatrix.at<float>(2, 5) = dt;

    // 预测
    cv::Mat prediction = kf_.predict();

    // 更新
    cv::Mat meas = (cv::Mat_<float>(3, 1) << measurement.x, measurement.y, measurement.z);
    cv::Mat estimated = kf_.correct(meas);

    // 提取位置和速度
    position_.x = estimated.at<float>(0);
    position_.y = estimated.at<float>(1);
    position_.z = estimated.at<float>(2);
    velocity_.x = estimated.at<float>(3);
    velocity_.y = estimated.at<float>(4);
    velocity_.z = estimated.at<float>(5);

    return position_;
}

cv::Point3f KalmanFilter::getPosition() const
{
    return position_;
}

cv::Point3f KalmanFilter::getVelocity() const
{
    return velocity_;
}

void KalmanFilter::reset()
{
    init();
}
