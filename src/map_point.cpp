#include "map_point.h"

namespace VISUAL_MAPPING {

    MapPoint::MapPoint(Eigen::Vector2d uv, Eigen::Vector3d x3D_, double cov_, int frame_id_, int feature_id, int id_) {
        features_uv.push_back(uv);
        x3D = x3D_;
        frame_ids.push_back(frame_id_);
        frame_id = frame_id_;
        id = id_;
        cov.push_back(cov_);
        min_cov = cov_;
        feature_ids.push_back(feature_id);
    }

    void MapPoint::add_frame(int frame_id, int feature_id, double cov_, Eigen::Vector2d uv, Eigen::Vector3d x3D_) {
        features_uv.push_back(uv);
        frame_ids.push_back(frame_id);
        cov.push_back(cov_);
        feature_ids.push_back(feature_id);
        if (min_cov > cov_) {
            // update 3D point
//            min_cov = cov_;
//            x3D = x3D_;
        }
    }

    int Map::add_map_point(std::shared_ptr<MapPoint> map_point) {
        // check if the map point already exists
        if (map_points.find(map_point->id) == map_points.end()) {
            map_points[map_point->id] = map_point;
            map_points_count++;
            max_id = std::max(max_id, map_point->id);
            return 1;
        }
        return 0;
    }

    void Map::remove_map_point(int id) {
        if (map_points.find(id) != map_points.end()) {
            map_points.erase(id);
            map_points_count--;
        }
    }

}

