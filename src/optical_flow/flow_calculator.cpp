#include "optical_flow/flow_calculator.h"
#include "optical_flow/feature_detector.h"
#include <algorithm>

class FlowCalculator::Impl {
public:
    Config config;
    FeatureDetector feature_detector;

    Impl(const Config &cfg) : config(cfg), feature_detector(FeatureDetector::Config{
        cfg.max_corners, cfg.quality_level, cfg.min_distance, cfg.block_size
        })
    {
    }
};

FlowCalculator::FlowCalculator()
    : impl_(std::make_unique<Impl>(Config()))
{
}

FlowCalculator::FlowCalculator(const Config &config)
    : impl_(std::make_unique<Impl>(config))
{
}

FlowCalculator::~FlowCalculator() {}

FlowCalculator::FlowResult FlowCalculator::calculate(
    const cv::Mat &prev_frame,
    const cv::Mat &curr_frame,
    const cv::Mat &reflection_mask,
    const cv::Mat &depth_map)
{

    (void)depth_map;
    FlowResult result;

    cv::Mat prev_gray, curr_gray;
    if (prev_frame.channels() == 3) {
        cv::cvtColor(prev_frame, prev_gray, cv::COLOR_BGR2GRAY);
        cv::cvtColor(curr_frame, curr_gray, cv::COLOR_BGR2GRAY);
    }
    else {
        prev_gray = prev_frame;
        curr_gray = curr_frame;
    }

    cv::Mat valid_mask;
    if (!reflection_mask.empty()) {
        cv::bitwise_not(reflection_mask, valid_mask);
    }

    result.prev_points = impl_->feature_detector.detect(prev_gray, valid_mask);

    if (result.prev_points.empty()) {
        result.confidence = 0.0f;
        result.valid_points_count = 0;
        return result;
    }

    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(
        prev_gray, curr_gray,
        result.prev_points, result.curr_points,
        result.status, err,
        impl_->config.win_size,
        impl_->config.max_level
    );

    result.flow_vectors.reserve(result.prev_points.size());
    std::vector<cv::Point2f> valid_flows;

    for (size_t i = 0; i < result.prev_points.size(); i++) {
        cv::Point2f flow = result.curr_points[i] - result.prev_points[i];
        result.flow_vectors.push_back(flow);

        if (result.status[i]) {
            float mag = std::sqrt(flow.x * flow.x + flow.y * flow.y);
            if (mag >= impl_->config.min_flow_magnitude &&
                mag <= impl_->config.max_flow_magnitude) {
                valid_flows.push_back(flow);
            }
        }
    }

    result.valid_points_count = static_cast<int>(valid_flows.size());

    if (!valid_flows.empty()) {
        cv::Point2f sum(0, 0);
        for (const auto &f : valid_flows) {
            sum += f;
        }
        result.average_flow = Eigen::Vector2f(sum.x / valid_flows.size(),
            sum.y / valid_flows.size());

        std::vector<float> x_flows, y_flows;
        for (const auto &f : valid_flows) {
            x_flows.push_back(f.x);
            y_flows.push_back(f.y);
        }
        std::sort(x_flows.begin(), x_flows.end());
        std::sort(y_flows.begin(), y_flows.end());
        size_t mid = valid_flows.size() / 2;
        result.median_flow = Eigen::Vector2f(x_flows[mid], y_flows[mid]);

        result.weighted_flow = result.median_flow;
        result.confidence = std::min(1.0f, static_cast<float>(valid_flows.size()) / 50.0f);
    }
    else {
        result.average_flow = Eigen::Vector2f::Zero();
        result.median_flow = Eigen::Vector2f::Zero();
        result.weighted_flow = Eigen::Vector2f::Zero();
        result.confidence = 0.0f;
    }

    return result;
}

cv::Mat FlowCalculator::visualize(const cv::Mat &frame, const FlowResult &result)
{
    cv::Mat vis = frame.clone();

    for (size_t i = 0; i < result.prev_points.size(); i++) {
        if (i < result.status.size() && result.status[i]) {
            cv::circle(vis, result.prev_points[i], 3, cv::Scalar(0, 255, 0), -1);
            cv::arrowedLine(vis, result.prev_points[i], result.curr_points[i],
                cv::Scalar(0, 255, 255), 1, cv::LINE_AA, 0, 0.3);
        }
    }

    std::string info = cv::format("Points: %d, Conf: %.2f",
        result.valid_points_count, result.confidence);
    cv::putText(vis, info, cv::Point(10, 30),
        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

    return vis;
}

void FlowCalculator::setConfig(const Config &config)
{
    impl_->config = config;
}
