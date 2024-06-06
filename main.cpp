#include <iostream>
#include <thread>
#include "frame.h"
#include "mapping.h"
#include "fstream"
#include "triangulation.h"
#include "bundle_adjustment.h"
#include "visualization.h"
#include "matcher.h"
#include "detection.h"
#include "camera.h"
#include "map_save.h"

// read yaml opencv
#include "opencv2/core.hpp"
#include "opencv2/core/types.hpp"
#include "opencv2/opencv.hpp"

using namespace VISUAL_MAPPING;

std::vector<std::pair<std::string, std::string>> read_img_path(const std::string& path1, const std::string& path2, const std::string& list, std::vector<Eigen::Matrix4d>& T){
    std::ifstream file;
    file.open(list);
    std::vector<std::pair<std::string, std::string>> img_list;
    std::string line;
    while (std::getline(file, line, ' ')) {
        img_list.emplace_back(path1 + line + ".png", path2 + line + ".png");
        Eigen::Matrix4d T_;
        Eigen::Vector3d t;
        Eigen::Quaterniond q;
        for (int i = 0;i < 6;i ++) {
            std::getline(file, line, ' ');
            if (i == 0) {
                t(0) = std::stod(line);
            } else if (i == 1) {
                t(1) = std::stod(line);
            } else if (i == 2) {
                t(2) = std::stod(line);
            } else if (i == 3) {
                q.x() = std::stod(line);
            } else if (i == 4) {
                q.y() = std::stod(line);
            } else if (i == 5) {
                q.z() = std::stod(line);
            }
        }
        std::getline(file, line, '\n');
        q.w() = std::stod(line);
        T_.block<3,3>(0,0) = q.toRotationMatrix();
        T_.block<3,1>(0,3) = t;
        T_.row(3) << 0, 0, 0, 1;
        T.push_back(T_);
    }
    return img_list;
}

void read_cam_params(const std::string& path, Camera& cam1, Camera& cam2, Eigen::Matrix4d& T12) {
    // yaml file
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "Failed to open " << path << std::endl;
        return;
    }
    std::string type = fs["Camera.type"];
    if (type == "pinhole") {
        double fx = fs["fx"];
    } else if (type == "KannalaBrandt8") {
        cam1.setModelType(KANNALA_BRANDT8);
        cam2.setModelType(KANNALA_BRANDT8);
        double fx = fs["Camera1.fx"];
        double fy = fs["Camera1.fy"];
        double cx = fs["Camera1.cx"];
        double cy = fs["Camera1.cy"];
        double k1 = fs["Camera1.k1"];
        double k2 = fs["Camera1.k2"];
        double k3 = fs["Camera1.k3"];
        double k4 = fs["Camera1.k4"];
        auto param2 = new Camera::KannalaBrandt8Params(fx, fy, cx, cy, k1, k2, k3, k4);
        cam1.setKannalaBrandt8Params(*param2);
        fx = fs["Camera2.fx"];
        fy = fs["Camera2.fy"];
        cx = fs["Camera2.cx"];
        cy = fs["Camera2.cy"];
        k1 = fs["Camera2.k1"];
        k2 = fs["Camera2.k2"];
        k3 = fs["Camera2.k3"];
        k4 = fs["Camera2.k4"];
        param2 = new Camera::KannalaBrandt8Params(fx, fy, cx, cy, k1, k2, k3, k4);
        cam2.setKannalaBrandt8Params(*param2);
    }
    cv::Mat T12_;
    fs["Stereo.T_c1_c2"] >> T12_;

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            float a = T12_.at<float>(i, j);
            T12(i, j) = a;
        }
    }
    fs.release();
}


int main() {
    // 1. read images list and camera parameters
    std::string img_path = "/home/vio/Datasets/TUM_VI/dataset-corridor2_512_16/mav0/cam0/data/";
    std::string img_path2 = "/home/vio/Datasets/TUM_VI/dataset-corridor2_512_16/mav0/cam1/data/";
    std::string img_list_path = "/home/vio/Code/VIO/ORB_SLAM3/Examples/Stereo/kf_corridor2.txt";
    std::vector<Eigen::Matrix4d> T;
    auto img_list = read_img_path(img_path, img_path2, img_list_path, T);

    std::string cam_param_path = "/home/vio/Code/VIO/ORB_SLAM3/Examples/Stereo/TUM-VI.yaml";
    Camera cam1, cam2;
    Eigen::Matrix4d T12;
    read_cam_params(cam_param_path, cam1, cam2, T12);

    std::string feature_list_path = "/home/vio/Dataset/features.yaml";
    std::vector<std::string> feature_list;

    // 2. create frame
    std::vector<std::shared_ptr<Frame>> frames;
    std::shared_ptr<FeatureDetection> detection =
            std::make_shared<FeatureDetection>(SuperPoint, "../learned_features_inference/weight/",
                                               6, 100, 512, 512);
    for (int i = 0; i < img_list.size(); i ++) {
        cv::Mat img1 = cv::imread(img_list[i].first);
        cv::Mat img2 = cv::imread(img_list[i].second);
        auto frame = std::make_shared<Frame>(i, detection, T[i], img1, img2, &cam1, &cam2, T12);
        std::cout<<"frame "<<i<<std::endl;
        frames.push_back(frame);
    }

    // 3. init mapping and refine
    Mapping mapping;
    mapping.construct_initial_map(frames);
    mapping.refine_map();

    // 4. save map
    MapSaver mapSaver;
    mapSaver.save_map("map.txt", frames, mapping.map);

//    MapSaver mapSaver;
//    Mapping mapping;
//    std::vector<Frame> frames;
//    mapSaver.load_map("map.txt", frames, mapping.map);

    // 5. visualization thread
    Visualization vis;
    std::thread vis_thread(Visualization::run, &vis, std::ref(mapping.map));
    vis_thread.join();

}
