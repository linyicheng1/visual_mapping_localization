#include "map_save.h"
#include <fstream>
#include <iostream>

namespace VISUAL_MAPPING {

    /**
     * @brief save map to file
     * @param filename
     * @param frames
     * @param map
     */
    void MapSaver::save_map(const std::string& filename, const std::vector<std::shared_ptr<Frame>> &frames, const Map &map) {
        std::fstream file;
        file.open(filename, std::ios::out);
        if (!file.is_open()) {
            std::cout << "Error: cannot open file " << filename << std::endl;
            return;
        }
        file <<frames.size()<<std::endl;
        for (const auto& frame : frames) {
            // id
            file << frame->id<<",";
            // pose
            file << frame->get_T()(0, 0) << "," << frame->get_T()(0, 1) << "," << frame->get_T()(0, 2) << "," << frame->get_T()(0, 3) << ",";
            file << frame->get_T()(1, 0) << "," << frame->get_T()(1, 1) << "," << frame->get_T()(1, 2) << "," << frame->get_T()(1, 3) << ",";
            file << frame->get_T()(2, 0) << "," << frame->get_T()(2, 1) << "," << frame->get_T()(2, 2) << "," << frame->get_T()(2, 3) << ",";
            file << frame->get_T()(3, 0) << "," << frame->get_T()(3, 1) << "," << frame->get_T()(3, 2) << "," << frame->get_T()(3, 3) << ",";
            // features
            file << frame->get_features_uv().size() << ",";
            for (const auto& uv : frame->get_features_uv()) {
                file << uv(0) << "," << uv(1) << ",";
            }
            // depth
            for (const auto& depth : frame->get_features_depth()) {
                file << depth << ",";
            }
            // descriptors
            file << frame->descriptors.rows << "," << frame->descriptors.cols << "," << frame->descriptors.type() << ",";
            for (int i = 0; i < frame->descriptors.rows; i++) {
                for (int j = 0; j < frame->descriptors.cols; j++) {
                    file << frame->descriptors.at<float>(i, j) << ",";
                }
            }
            // map points
            for (const auto& map_point : frame->map_points) {
                if (map_point != nullptr) {
                    file << map_point->id << ",";
                } else {
                    file << "-1,";
                }
            }
            file << std::endl;
        }
        // map points
        int num_map_points = 0;
        for (int i = 0; i < map.max_id; i++) {
            if (map.map_points.find(i) == map.map_points.end()) {
                continue;
            }
            num_map_points ++;
        }
        file << num_map_points << std::endl;
        for (int i = 0; i < map.max_id; i++) {
            if (map.map_points.find(i) == map.map_points.end()) {
                continue;
            }
            auto map_point = map.map_points.at(i);
            if (map_point != nullptr) {
                file << map_point->id << ",";
                file << map_point->x3D(0) << "," << map_point->x3D(1) << "," << map_point->x3D(2) << ",";
                file << map_point->frame_ids.size() << ",";
                for (const auto& frame_id : map_point->frame_ids) {
                    file << frame_id << ",";
                }
                file << map_point->features_uv.size() << ",";
                for (const auto& uv : map_point->features_uv) {
                    file << uv(0) << "," << uv(1) << ",";
                }
                for (const auto& id : map_point->feature_ids) {
                    file << id << ",";
                }
            }
            file << std::endl;
        }
        file.close();
    }

    void MapSaver::load_map(const std::string &filename, std::vector<std::shared_ptr<Frame>> &frames, Map &map) {
        std::fstream file;
        file.open(filename, std::ios::in);
        if (!file.is_open()) {
            std::cout << "Error: cannot open file " << filename << std::endl;
            return;
        }
        int frame_count;
        std::string line;
        std::getline(file, line);
        frame_count = std::stoi(line);
        for (int i = 0; i < frame_count; i++) {
            std::getline(file, line);
            std::stringstream ss(line);
            std::string token;
            std::vector<std::string> tokens;
            while (std::getline(ss, token, ',')) {
                tokens.push_back(token);
            }
            auto frame = std::make_shared<Frame>();
            frame->id = std::stoi(tokens[0]);
            Eigen::Matrix4d T;
            T << std::stod(tokens[1]), std::stod(tokens[2]), std::stod(tokens[3]), std::stod(tokens[4]),
                    std::stod(tokens[5]), std::stod(tokens[6]), std::stod(tokens[7]), std::stod(tokens[8]),
                    std::stod(tokens[9]), std::stod(tokens[10]), std::stod(tokens[11]), std::stod(tokens[12]),
                    std::stod(tokens[13]), std::stod(tokens[14]), std::stod(tokens[15]), std::stod(tokens[16]);
            frame->set_T(T);
            int feature_count = std::stoi(tokens[17]);
            for (int j = 0; j < feature_count; j++) {
                Eigen::Vector2d uv(std::stod(tokens[18 + 2 * j]), std::stod(tokens[18 + 2 * j + 1]));
                frame->features_uv.push_back(uv);
            }
            for (int j = 0; j < feature_count; j++) {
                frame->features_depth.push_back(std::stod(tokens[18 + 2 * feature_count + j]));
            }
            // descriptors
            int rows = std::stoi(tokens[18 + 3 * feature_count]);
            int cols = std::stoi(tokens[18 + 3 * feature_count + 1]);
            int type = std::stoi(tokens[18 + 3 * feature_count + 2]);
            cv::Mat descriptors(rows, cols, type);
            for (int j = 0; j < rows; j++) {
                for (int k = 0; k < cols; k++) {
                    if (type == CV_8U) {
                        descriptors.at<uchar>(j, k) = std::stoi(tokens[18 + 3 * feature_count + 3 + j * cols + k]);
                    } else if (type == CV_32F) {
                        descriptors.at<float>(j, k) = std::stof(tokens[18 + 3 * feature_count + 3 + j * cols + k]);
                    }
                }
            }
            frame->descriptors = descriptors;
            frame->map_points.resize(feature_count, nullptr);
            frames.push_back(frame);
        }
        // map points
        std::getline(file, line);
        int map_point_count = std::stoi(line);
        for (int i = 0; i < map_point_count; i++) {
            std::getline(file, line);
            std::stringstream ss(line);
            if (line.empty()) {
                continue;
            }
            std::string token;
            std::vector<std::string> tokens;
            while (std::getline(ss, token, ',')) {
                tokens.push_back(token);
            }
            int id = std::stoi(tokens[0]);
            Eigen::Vector3d x3D(std::stod(tokens[1]), std::stod(tokens[2]), std::stod(tokens[3]));
            int frame_counts = std::stoi(tokens[4]);
            std::vector<int> frame_ids;
            for (int j = 0; j < frame_counts; j++) {
                frame_ids.push_back(std::stoi(tokens[5 + j]));
            }
            int feature_count = std::stoi(tokens[5 + frame_counts]);
            std::vector<Eigen::Vector2d> features_uv;
            for (int j = 0; j < feature_count; j++) {
                Eigen::Vector2d uv(std::stod(tokens[6 + frame_counts + 2 * j]), std::stod(tokens[6 + frame_counts + 2 * j + 1]));
                features_uv.push_back(uv);
            }
            std::vector<int> feature_ids;
            for (int j = 0; j < feature_count; j++) {
                feature_ids.push_back(std::stoi(tokens[6 + frame_counts + 2 * feature_count + j]));
            }
            std::shared_ptr<MapPoint> map_point = std::make_shared<MapPoint>(features_uv[0], x3D, 0.1, frame_ids[0], feature_ids[0], id);
            for (int j = 1; j < frame_counts; j++) {
                map_point->add_frame(frame_ids[j], feature_ids[j], 0.2, features_uv[j], x3D);
            }
            map_point->descriptor = frames[frame_ids[0]]->descriptors.row(feature_ids[0]);
            map.add_map_point(map_point);
            for (int j = 0;j < feature_count; j++) {
                frames[frame_ids[j]]->map_points[feature_ids[j]] = map_point;
            }

            // link frames
            for (int j = 0; j < frame_counts; j++) {
                for (int k = j+1; k < frame_counts; k++) {
                    frames[frame_ids[j]]->add_linked_frame(frames[frame_ids[k]]);
                    frames[frame_ids[k]]->add_linked_frame(frames[frame_ids[j]]);
                }
            }
        }
    }
}
