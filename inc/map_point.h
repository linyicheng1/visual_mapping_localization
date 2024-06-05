#ifndef __MAP_POINT_H_
#define __MAP_POINT_H_

#include <unordered_map>
#include <Eigen/Core>
#include <vector>
#include "memory"
#include "opencv2/core.hpp"

namespace VISUAL_MAPPING {


    class MapPoint {
    public:
        MapPoint() = default;
        ~MapPoint() = default;
        MapPoint(Eigen::Vector2d uv, Eigen::Vector3d x3D, double cov, int frame_id, int feature_id, int id);
        void add_frame(int frame_id, int feature_id, double parallax, Eigen::Vector2d uv, Eigen::Vector3d x3D_);

        Eigen::Vector3d x3D;
        std::vector<Eigen::Vector2d> features_uv;
        std::vector<int> frame_ids;
        std::vector<int> feature_ids;
        int id;
        int frame_id;
        std::vector<double> cov;
        double min_cov;
        cv::Mat descriptor;
    };

    class Map {
     public:
        Map() = default;
        ~Map() = default;
        std::shared_ptr<MapPoint> get_map_point(int id) { return map_points[id];}
        int add_map_point(std::shared_ptr<MapPoint> map_point);
        void remove_map_point(int id);

        int map_points_count = 0;
        int max_id = 0;

        std::unordered_map<int, std::shared_ptr<MapPoint>> map_points;


    };
}

#endif //__MAP_POINT_H_
