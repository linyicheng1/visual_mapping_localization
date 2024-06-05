#include "frame.h"
#include "detection.h"
#include "matcher.h"
#include "triangulation.h"
#include <iostream>

namespace VISUAL_MAPPING {

    Frame::Frame(cv::Mat &image, std::vector<Eigen::Vector2d> &features_uv,
                           std::vector<double> &features_depth, cv::Mat &descriptors, Camera *camera) {


    }

    Frame::Frame(int id_, Eigen::Matrix4d T, cv::Mat &image_left, cv::Mat &image_right,
                 Camera *cam_left, Camera *cam_right, Eigen::Matrix4d &T12) {
        image = image_left;
        id = id_;
        camera = cam_left;
        R = T.block<3, 3>(0, 0);
        t = T.block<3, 1>(0, 3);
        // 1. detect features
        Detection detector;
        detector.detectFeatures(image_left, features_uv, descriptors);
        std::vector<Eigen::Vector2d> features_right;
        cv::Mat descriptors_right;
        detector.detectFeatures(image_right, features_right, descriptors_right);
        // 2. compute depth
        Matcher matcher;
        auto matches = matcher.match_stereo(image_left, image_right, features_uv, features_right, descriptors, descriptors_right);
        double baseline = T12.block<3, 1>(0, 3).norm();
        features_depth.clear();
        features_depth.resize(features_uv.size(), -1);
        features_depth_cov.clear();
        features_depth_cov.resize(features_uv.size(), -1);

        map_points.clear();
        map_points.resize(features_uv.size(), nullptr);
        Triangulation triangulation;
        int cnt = 0;
        for (auto &match : matches) {
            std::pair<double, double> depth= triangulation.triangulate(cam_left, cam_right,
                                                    features_uv[match.first],
                                                    features_right[match.second],
                                                    T12.block<3, 3>(0, 0),
                                                    T12.block<3, 1>(0, 3));
            features_depth[match.first] = depth.first;
            features_depth_cov[match.first] = depth.second;
            if (depth.first > 0) {
                cnt++;
            }
        }
        assign_features_to_grid();
//        std::cout << "Triangulated " << cnt << " points" << std::endl;
    }


    std::vector<int> Frame::get_around_features(Eigen::Vector2d uv, double radius) {
        const int cell = 10;
        const int cell_x = int(uv[0] / (float)cell);
        const int cell_y = int(uv[1] / (float)cell);
        const int cell_radius = int(radius / (float)cell);
        std::vector<int> result;

        for (int x = std::max(0, cell_x - cell_radius); x < std::min(200, cell_x + cell_radius); x++) {
            for (int y = std::max(0, cell_y - cell_radius); y < std::min(200, cell_y + cell_radius); y++) {
                result.insert(result.end(), mGrid[x][y].begin(), mGrid[x][y].end());
            }
        }
        return result;
    }

    void Frame::assign_features_to_grid() {
        const int cell = 10;
        for (int i = 0; i < features_uv.size(); i++) {
            Eigen::Vector2d uv = features_uv[i];
            int x = int(uv[0] / (float)cell);
            int y = int(uv[1] / (float)cell);
            if (x < 0 || x >= 200 || y < 0 || y >= 200) {
                continue;
            }
            mGrid[x][y].push_back(i);
        }
    }

    void Frame::add_linked_frame(Frame *frame) {
        for (auto &f : related_frames) {
            if (f->id == frame->id) {
                return;
            }
        }
        related_frames.push_back(frame);
    }
}
