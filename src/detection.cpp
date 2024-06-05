#include "detection.h"
#include "opencv2/features2d.hpp"

namespace VISUAL_MAPPING {

    void Detection::detectFeatures(cv::Mat &image, std::vector<Eigen::Vector2d> &features_uv, cv::Mat &descriptors) {
        // 1. detect sift features
        auto sift = cv::SIFT::create();
        std::vector<cv::KeyPoint> keypoints;
        sift->detect(image, keypoints);
        for (auto &keypoint : keypoints) {
            features_uv.emplace_back(keypoint.pt.x, keypoint.pt.y);
        }
        // 2. compute descriptors
        sift->compute(image, keypoints, descriptors);
    }

} // namespace reusable_map
