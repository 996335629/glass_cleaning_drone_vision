#include "optical_flow/depth_fusion.h"

DepthFusion::DepthFusion() : config_(Config()) {}
DepthFusion::DepthFusion(const Config &config) : config_(config) {}

DepthFusion::~DepthFusion() {}

std::vector<float> DepthFusion::getDepthsAtPoints(const cv::Mat &depth_map,
    const std::vector<cv::Point2f> &points)
{
    std::vector<float> depths;
    depths.reserve(points.size());

    for (const auto &pt : points) {
        int x = static_cast<int>(pt.x);
        int y = static_cast<int>(pt.y);

        if (x >= 0 && x < depth_map.cols && y >= 0 && y < depth_map.rows) {
            float depth = 0.0f;
            if (depth_map.type() == CV_16U) {
                depth = depth_map.at<uint16_t>(y, x) / config_.depth_scale;
            }
            else if (depth_map.type() == CV_32F) {
                depth = depth_map.at<float>(y, x);
            }

            if (depth < config_.min_depth || depth > config_.max_depth) {
                depth = 0.0f;
            }
            depths.push_back(depth);
        }
        else {
            depths.push_back(0.0f);
        }
    }

    return depths;
}

Eigen::Vector3f DepthFusion::flowToVelocity(const Eigen::Vector2f &flow,
    float depth,
    const cv::Mat &camera_matrix)
{
    if (depth <= 0.0f || camera_matrix.empty()) {
        return Eigen::Vector3f::Zero();
    }

    float fx = static_cast<float>(camera_matrix.at<double>(0, 0));
    float fy = static_cast<float>(camera_matrix.at<double>(1, 1));

    float vx = flow.x() * depth / fx;
    float vy = flow.y() * depth / fy;

    return Eigen::Vector3f(vx, vy, 0.0f);
}
