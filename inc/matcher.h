#ifndef __MAP_MATCHER_H_
#define __MAP_MATCHER_H_
#include "frame.h"
#include "opencv2/core.hpp"

namespace VISUAL_MAPPING {

    class Matcher {
    public:
        Matcher() = default;
        // brute force matching
        void match_bf(Frame* frame1, Frame* frame2, std::vector<cv::DMatch>& matches);
        // epipolar constraint matching
        void match_epipolar(Frame* frame1, Frame* frame2, std::vector<cv::DMatch>& matches);
        // projective constraint matching
        std::vector<std::pair<int, int>> match_re_projective(std::shared_ptr<Frame> frame1, std::shared_ptr<Frame> frame2);
        std::vector<std::pair<int, int>> match_projective(Frame& frame1, std::vector<std::shared_ptr<MapPoint>> map_points);
        std::vector<std::pair<int, int>> match_stereo(cv::Mat img1, cv::Mat img2,
                                                      std::vector<Eigen::Vector2d>& features_uv_left,
                                                      std::vector<Eigen::Vector2d>& features_uv_right,
                                                      cv::Mat& descriptors_left, cv::Mat& descriptors_right);
        std::vector<std::pair<int, int>> match_knn(Frame& frame1, Frame& frame2);
    };
}

#endif //__MAP_MATCHER_H_
