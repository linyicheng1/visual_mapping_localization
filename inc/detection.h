#ifndef __MAP_DETECTION_H_
#define __MAP_DETECTION_H_
#include "opencv2/core.hpp"
#include "Eigen/Core"

namespace VISUAL_MAPPING {

    class Detection {
    public:
        Detection() = default;
        void detectFeatures(cv::Mat& image, std::vector<Eigen::Vector2d>& features_uv, cv::Mat& descriptors);
    };

}

#endif //__MAP_DETECTION_H_
