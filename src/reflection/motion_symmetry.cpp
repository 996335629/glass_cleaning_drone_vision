#include "reflection/motion_symmetry.h"
#include <cmath>

MotionSymmetryDetector::MotionSymmetryDetector() : config_(Config()) {}

MotionSymmetryDetector::MotionSymmetryDetector(const Config &config)
    : config_(config)
{
}

MotionSymmetryDetector::~MotionSymmetryDetector() {}

std::vector<MotionSymmetryDetector::SymmetryPair>
MotionSymmetryDetector::detectSymmetry(const std::vector<cv::Point2f> &points,
    const std::vector<cv::Point2f> &motions)
{
    std::vector<SymmetryPair> pairs;

    if (points.size() != motions.size() || points.empty()) {
        return pairs;
    }

    for (size_t i = 0; i < points.size(); i++) {
        for (size_t j = i + 1; j < points.size(); j++) {
            float dot = motions[i].x * motions[j].x + motions[i].y * motions[j].y;
            float mag1 = std::sqrt(motions[i].x * motions[i].x + motions[i].y * motions[i].y);
            float mag2 = std::sqrt(motions[j].x * motions[j].x + motions[j].y * motions[j].y);

            if (mag1 < 0.1f || mag2 < 0.1f) continue;

            float cos_angle = dot / (mag1 * mag2);

            if (cos_angle < -0.7f) {
                float mag_ratio = std::min(mag1, mag2) / std::max(mag1, mag2);
                if (mag_ratio > (1.0f - static_cast<float>(config_.magnitude_tolerance))) {
                    SymmetryPair pair;
                    pair.point1 = points[i];
                    pair.point2 = points[j];
                    pair.motion1 = motions[i];
                    pair.motion2 = motions[j];
                    pair.symmetry_score = mag_ratio * (1.0f + cos_angle);
                    pairs.push_back(pair);
                }
            }
        }
    }

    return pairs;
}

cv::Mat MotionSymmetryDetector::visualize(const cv::Mat &frame,
    const std::vector<SymmetryPair> &pairs)
{
    cv::Mat vis = frame.clone();

    for (const auto &pair : pairs) {
        cv::line(vis, pair.point1, pair.point2, cv::Scalar(255, 0, 255), 1);
        cv::circle(vis, pair.point1, 4, cv::Scalar(0, 255, 0), -1);
        cv::circle(vis, pair.point2, 4, cv::Scalar(0, 0, 255), -1);
    }

    return vis;
}
