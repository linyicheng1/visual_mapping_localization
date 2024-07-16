#include "detection.h"
#include "opencv2/features2d.hpp"
#include "extractor.h"
#include "interface.h"

namespace VISUAL_MAPPING {

    FeatureDetection::FeatureDetection(int type, const std::string& weights_path, int nms, int num_kps, int width, int height) {
        this->type = type;
        this->weights_path = weights_path;
        this->nms_size = nms;
        this->num_kps = num_kps;
        this->IMG_WIDTH = width;
        this->IMG_HEIGHT = height;
        if (type == ALIKE) {
            net_ptr = std::make_shared<Interface>("alike", weights_path + alike_model_path, true, cv::Size(IMG_WIDTH,IMG_HEIGHT));
            descriptor_dim = 64;
            descriptor_width = IMG_WIDTH;
            descriptor_height = IMG_HEIGHT;
        } else if (type == D2Net) {
            net_ptr = std::make_shared<Interface>("d2net", weights_path + d2net_model_path, true, cv::Size(IMG_WIDTH,IMG_HEIGHT));
            descriptor_dim = 512;
            descriptor_width = IMG_WIDTH / 8;
            descriptor_height = IMG_HEIGHT / 8;
        } else if (type == SuperPoint) {
            net_ptr = std::make_shared<Interface>("SuperPoint", weights_path + sp_model_path, true, cv::Size(IMG_WIDTH,IMG_HEIGHT));
            descriptor_dim = 256;
            descriptor_width = IMG_WIDTH / 8;
            descriptor_height = IMG_HEIGHT / 8;
        } else if (type == DISK) {
            net_ptr = std::make_shared<Interface>("disk", weights_path + disk_model_path, true, cv::Size(IMG_WIDTH,IMG_HEIGHT));
            descriptor_dim = 128;
            descriptor_width = IMG_WIDTH;
            descriptor_height = IMG_HEIGHT;
        } else if (type == XFeat) {
            net_ptr = std::make_shared<Interface>("xfeat", weights_path + xfeat_model_path, true, cv::Size(IMG_WIDTH,IMG_HEIGHT));
            descriptor_dim = 64;
            descriptor_width = IMG_WIDTH / 8;
            descriptor_height = IMG_HEIGHT / 8;
        }
        score_map = cv::Mat(IMG_HEIGHT, IMG_WIDTH, CV_32FC1);
        desc_map = cv::Mat(descriptor_height, descriptor_width, CV_32FC(descriptor_dim));
    }

    void FeatureDetection::detectFeatures(cv::Mat &image, std::vector<Eigen::Vector2d> &features_uv, cv::Mat &descriptors) {
        cv::Mat mask = cv::Mat::ones(image.size(), CV_8UC1) * 255;
        // 最下方100行不检测特征点
        mask(cv::Rect(0, image.rows - 100, image.cols, 100)) = 0;

        if (type == SIFT) {
            // 1. detect sift features
            auto sift = cv::SIFT::create();
            std::vector<cv::KeyPoint> keypoints;
            sift->detect(image, keypoints);
            for (auto &keypoint : keypoints) {
                features_uv.emplace_back(keypoint.pt.x, keypoint.pt.y);
            }
            // 2. compute descriptors
            sift->compute(image, keypoints, descriptors);
        } else if (type == ORB) {
            // 1. detect orb features
            auto orb = cv::ORB::create();
            std::vector<cv::KeyPoint> keypoints;
            orb->detect(image, keypoints);
            for (auto &keypoint : keypoints) {
                features_uv.emplace_back(keypoint.pt.x, keypoint.pt.y);
            }
            // 2. compute descriptors
            orb->compute(image, keypoints, descriptors);
        } else if (type == SuperPoint || type == ALIKE || type == D2Net || type == DISK || type == XFeat) {
            net_ptr->run(image, score_map, desc_map);
            std::vector<cv::KeyPoint> key_points;
            key_points = nms(score_map, num_kps, 0.01, nms_size, mask);
            for (auto &keypoint : key_points) {
                features_uv.emplace_back(keypoint.pt.x, keypoint.pt.y);
            }
            descriptors = bilinear_interpolation(image.cols, image.rows, desc_map, key_points);
        } else {
            std::cout<<"model type not supported"<<std::endl;
        }
    }

} // namespace reusable_map
