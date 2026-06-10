#include "simulator/reflection_simulator.h"

ReflectionSimulator::ReflectionSimulator() : config_(Config()) {}
ReflectionSimulator::ReflectionSimulator(const Config &config) : config_(config) {}

ReflectionSimulator::~ReflectionSimulator() {}

cv::Mat ReflectionSimulator::generateReflection(const cv::Mat &drone_image,
    const cv::Mat &background)
{
    (void)background;

    cv::Mat reflection = drone_image.clone();

    if (config_.blur_strength > 0) {
        int ksize = static_cast<int>(config_.blur_strength) * 2 + 1;
        cv::GaussianBlur(reflection, reflection, cv::Size(ksize, ksize), config_.blur_strength);
    }

    reflection *= config_.reflectivity;

    return reflection;
}

void ReflectionSimulator::setConfig(const Config &config)
{
    config_ = config;
}
