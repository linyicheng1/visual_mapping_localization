#ifndef __FRAME_H_
#define __FRAME_H_
#include "camera.h"
#include "opencv2/core.hpp"
#include "map_point.h"
#include "detection.h"

namespace VISUAL_MAPPING {

    class Frame {
    public:
        Frame() = default;
        ~Frame() = default;
        Frame(int id_, std::shared_ptr<FeatureDetection> detection, Eigen::Matrix4d T, cv::Mat& image_left, cv::Mat& image_right, Camera* cam_left, Camera* cam_right, Eigen::Matrix4d& T12);
        Frame(cv::Mat& image, std::vector<Eigen::Vector2d>& features_uv, std::vector<double>& features_depth, cv::Mat& descriptors, Camera* camera);
        Eigen::Matrix3d get_R() const { return R;}
        Eigen::Vector3d get_t() const { return t;}
        Eigen::Matrix4d get_T() const { Eigen::Matrix4d T; T.block<3, 3>(0, 0) = R; T.block<3, 1>(0, 3) = t; T.row(3) << 0, 0, 0, 1; return T;}
        void set_T(Eigen::Matrix4d T) { R = T.block<3, 3>(0, 0); t = T.block<3, 1>(0, 3);}
        std::vector<Eigen::Vector2d> get_features_uv() const { return features_uv;}
        std::vector<double> get_features_depth() const { return features_depth;}
        std::vector<int> get_around_features(Eigen::Vector2d uv, double radius);
        Camera* get_camera() const { return camera;}
        std::vector<Frame*> get_connected_frames() const { return related_frames;}
        void add_linked_frame(Frame* frame);

        int id;
        std::vector<std::shared_ptr<MapPoint>> map_points;
        std::vector<double> features_depth; // depth in meters (z)
        std::vector<double> features_depth_cov; // depth uncertainty
        cv::Mat descriptors;
        cv::Mat image;
        // features
        std::vector<Eigen::Vector2d> features_uv;
        Camera* camera;
    private:

        // frame pose
        Eigen::Matrix3d R;
        Eigen::Vector3d t;

        // related frames
        std::vector<Frame*> related_frames;

        void assign_features_to_grid();
        std::vector<std::size_t> mGrid[200][200];
        std::shared_ptr<FeatureDetection> detector_ptr;
    };
}

#endif //__FRAME_H_
