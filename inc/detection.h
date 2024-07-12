#ifndef __MAP_DETECTION_H_
#define __MAP_DETECTION_H_
#include "opencv2/core.hpp"
#include "Eigen/Core"
#include "interface.h"

namespace VISUAL_MAPPING {

    enum {
        ORB = 0,
        SIFT = 1,
        SuperPoint = 2,
        ALIKE = 3,
        D2Net = 4,
        DISK = 5,
        XFeat = 6,
    };

    class FeatureDetection {
    public:
        FeatureDetection() = default;
        FeatureDetection(int type, const std::string& weights_path, int nms, int num_kps, int width, int height);
        ~FeatureDetection() = default;
        void detectFeatures(cv::Mat& image, std::vector<Eigen::Vector2d>& features_uv, cv::Mat& descriptors);

    private:
        int type{};
        int nms_size = 6;
        int num_kps = 1000;
        int IMG_WIDTH = 640;
        int IMG_HEIGHT = 480;

        std::string weights_path = "./weights/";
        std::string sp_model_path = "sp_84.xml";
        std::string xfeat_model_path = "xfeat.xml";
        std::string d2net_model_path = "D2Net.xml";
        std::string disk_model_path = "disk.xml";
        std::string alike_model_path = "Alike.xml";

        std::shared_ptr<Interface> net_ptr;
        int descriptor_dim = 64;
        int descriptor_width = IMG_WIDTH;
        int descriptor_height = IMG_HEIGHT;

        cv::Mat score_map;
        cv::Mat desc_map;
    };

}

#endif //__MAP_DETECTION_H_
