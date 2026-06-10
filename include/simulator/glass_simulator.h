#pragma once

#include <opencv2/opencv.hpp>
#include <memory>

class GlassSimulator {
public:
    struct GlassProperties {
        float reflectivity = 0.6f;
        float blur_strength = 5.0f;
    };

    struct SceneConfig {
        cv::Size frame_size = cv::Size(640, 480);
        GlassProperties glass;
        bool add_noise = true;
    };

    GlassSimulator(const SceneConfig &config = SceneConfig());
    ~GlassSimulator();

    void generateFrame(cv::Mat &left_frame, cv::Mat &right_frame, cv::Mat &depth_map);
    void updateDroneMotion(const cv::Point2f &velocity, float angular_velocity);
    void addRandomDisturbance();
    cv::Point2f getGroundTruthDisplacement() const;
    void reset();

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};
