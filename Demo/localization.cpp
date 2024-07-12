#include "map_save.h"
#include "visualization.h"
#include <thread>
#include <opencv2/opencv.hpp>
#include "mapping.h"
#include "matcher.h"
#include "bundle_adjustment.h"
#include "camera.h"

using namespace VISUAL_MAPPING;
#define IMAGE_WIDTH 800
#define IMAGE_HEIGHT 400

class Localization {
public:
    Localization() = default;
    ~Localization() = default;
    std::vector<int> sort_frames_by_distance(std::vector<std::shared_ptr<Frame>>& frames, Eigen::Matrix3d R, Eigen::Vector3d t);
    std::vector<int> sort_frames_by_distance(std::vector<std::shared_ptr<Frame>>& frames, Eigen::Matrix4d T) {
        return sort_frames_by_distance(frames, T.block<3, 3>(0, 0), T.block<3, 1>(0, 3));
    }
};

std::vector<int> Localization::sort_frames_by_distance(std::vector<std::shared_ptr<Frame>> &frames, Eigen::Matrix3d R, Eigen::Vector3d t) {
    std::vector<int> sorted_ids;
    std::vector<double> sorted_distances;
    sorted_ids.reserve(frames.size());
    sorted_distances.reserve(frames.size());
    for (const auto& frame : frames) {
        Eigen::Vector3d t_ = frame->get_t();
        Eigen::Vector3d t_diff = t_ - t;
        double distance = t_diff.norm();
        sorted_distances.push_back(distance);
        sorted_ids.push_back(frame->id);
    }
    std::sort(sorted_ids.begin(), sorted_ids.end(), [&sorted_distances](int i, int j) {
        return sorted_distances[i] < sorted_distances[j];
    });
    return sorted_ids;
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

std::vector<std::string> read_img_path(const std::string& path, const std::string& list) {
    std::ifstream file;
    file.open(list);
    std::vector<std::string> img_list;
    std::string line;
    while (std::getline(file, line, '\n')) {
        img_list.emplace_back(path + line + ".png");
//        Eigen::Matrix4d T_;
//        Eigen::Vector3d t;
//        Eigen::Quaterniond q;
//        for (int i = 0;i < 6;i ++) {
//            std::getline(file, line, ' ');
//            if (i == 0) {
//                t(0) = std::stod(line);
//            } else if (i == 1) {
//                t(1) = std::stod(line);
//            } else if (i == 2) {
//                t(2) = std::stod(line);
//            } else if (i == 3) {
//                q.x() = std::stod(line);
//            } else if (i == 4) {
//                q.y() = std::stod(line);
//            } else if (i == 5) {
//                q.z() = std::stod(line);
//            }
//        }
//        std::getline(file, line, '\n');
//        q.w() = std::stod(line);
//        T_.block<3,3>(0,0) = q.toRotationMatrix();
//        T_.block<3,1>(0,3) = t;
//        T_.row(3) << 0, 0, 0, 1;
    }
    return img_list;
}

std::vector<std::string> read_img_path_kf(const std::string& path1, const std::string& list){
    std::ifstream file;
    file.open(list);
    std::vector<std::string> img_list;
    std::string line;
    while (std::getline(file, line, ' ')) {
        img_list.emplace_back(path1 + line + ".png");
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
    }
    return img_list;
}

int main() {
    MapSaver map_saver;
    std::vector<std::shared_ptr<Frame>> frames;
    Map map;
    Matcher matcher;
    Localization localization;
    map_saver.load_map("map.txt", frames, map);

    Visualization vis;
    std::thread vis_thread(Visualization::run, &vis, std::ref(map));

    Camera cam1, cam2;
    Eigen::Matrix4d T12;
    std::string cam_param_path = "/home/vio/Code/VIO/visual_localization/ORB_SLAM3_localization/Examples/Stereo/4seasons.yaml";
    read_cam_params(cam_param_path, cam1, cam2, T12);
    for (auto& frame : frames) {
        frame->camera = &cam1;
    }
    std::string img_path = "/home/vio/Datasets/4seasons/recording_2021-05-10_19-15-19/undistorted_images/cam0/";
    std::string img_list_path = "/home/vio/Datasets/4seasons/recording_2021-05-10_19-15-19/times-0.txt";

    std::string img_path_kf = "/home/vio/Datasets/4seasons/recording_2020-12-22_12-04-35/undistorted_images/cam0/";
    std::string img_list_path_kf = "/home/vio/Code/VIO/visual_localization/ORB_SLAM3_localization/Examples/Stereo/kf_corridor2_02.txt";

    auto img_list = read_img_path(img_path, img_list_path);
    auto img_list_kf = read_img_path_kf(img_path_kf, img_list_path_kf);

    Eigen::Matrix4d I = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d init_T = I;
//    img_list.resize(200);
    std::shared_ptr<FeatureDetection> detection =
            std::make_shared<FeatureDetection>(SuperPoint, "../learned_features_inference/weight/",
                                               8, 1000, 800, 400);
    int cnt = 0;
    for (auto & img_cnt : img_list) {
        cnt ++;
        if (cnt < 500)
            continue;
        std::cout<<"cnt: "<<cnt<<std::endl;
        // 1. get the first frame
        cv::Mat img1 = cv::imread(img_cnt);
        std::cout<<"init T: "<<init_T<<std::endl;
        std::shared_ptr<Frame> tgt_frame = std::make_shared<Frame>(0, detection, init_T, img1, img1, &cam1, &cam1, I);

        // 2. get close frames
        std::vector<int> close_frame_ids = localization.sort_frames_by_distance(frames, init_T);
        cv::Mat img_kf = cv::imread(img_list_kf[close_frame_ids[0]+157]);
        std::shared_ptr<Frame> kf_frame = std::make_shared<Frame>(0, detection, init_T, img_kf, img_kf, &cam1, &cam1, I);
        std::vector<std::pair<int, int>> matches_kf = matcher.match_knn(*tgt_frame, *kf_frame);
        cv::Mat img_matches = cv::Mat(IMAGE_HEIGHT, IMAGE_WIDTH * 2, CV_8UC3);
        img1.copyTo(img_matches(cv::Rect(0, 0, IMAGE_WIDTH, IMAGE_HEIGHT)));
        img_kf.copyTo(img_matches(cv::Rect(IMAGE_WIDTH, 0, IMAGE_WIDTH, IMAGE_HEIGHT)));

        for (const auto m:matches_kf) {
            Eigen::Vector2d pt1 = tgt_frame->get_features_uv()[m.first];
            Eigen::Vector2d pt2 = kf_frame->get_features_uv()[m.second];
            cv::line(img_matches, cv::Point2f((float)pt2.x(), (float)pt2.y()),
                     cv::Point2f((float)pt2.x() + IMAGE_WIDTH, (float)pt2.y()),
                     cv::Scalar(0, 255, 0), 2);
        }
        cv::imshow("matches", img_matches);

        // 3. try match the closest 3 frames
        for (int i = 0;i < 5;i ++) {
            std::shared_ptr<Frame> frame = frames[close_frame_ids[i]];
            tgt_frame->map_points.resize(tgt_frame->map_points.size(), nullptr);
            // 3.1 KNN match
            std::vector<std::pair<int, int>> matches = matcher.match_knn(*tgt_frame, *frame);
            int matches_num = 0;
            for (auto &match : matches) {
                if (frame->map_points[match.second] != nullptr) {
                    tgt_frame->map_points[match.first] = frame->map_points[match.second];
                    matches_num ++;
                }
            }
            // 3.3 PNP
            BundleAdjustment ba;
            if (matches_num > 30) {
                tgt_frame->set_T(init_T);
                std::vector<bool> inliers = ba.optimize_pose(tgt_frame);
//                std::cout<<"T " << tgt_frame->get_T() << std::endl;;
                int inliers_num = 0;
                for (int j = 0;j < inliers.size();j ++) {
                    if (!inliers[j]) {
                        tgt_frame->map_points[j] = nullptr;
                    } else {
                        inliers_num ++;
                    }
                }
            }

            matches = matcher.match_re_projective(tgt_frame, frame);
            matches_num = 0;
            for (auto &match : matches) {
                if (frame->map_points[match.second] != nullptr) {
                    tgt_frame->map_points[match.first] = frame->map_points[match.second];
                    matches_num ++;
                }
            }
            if (matches_num > 30) {
                tgt_frame->set_T(init_T);
                std::vector<bool> inliers = ba.optimize_pose(tgt_frame);
//                std::cout<<"T " << tgt_frame->get_T() << std::endl;;
                int inliers_num = 0;
                for (int j = 0;j < inliers.size();j ++) {
                    if (!inliers[j]) {
                        tgt_frame->map_points[j] = nullptr;
                    } else {
                        inliers_num ++;
                    }
                }
                if (inliers_num > 30) {
                    std::cout<<" second inliers: " << inliers_num << std::endl;
                    break;
                }
            }
        }

//        std::vector<std::shared_ptr<Frame>> connected_frames_1;
//        connected_frames_1 = frames[close_frame_ids[0]]->get_connected_frames();
//
//        int add_num = 0;
//        for (auto & frame : connected_frames_1) {
//            std::vector<std::pair<int, int>> matches = matcher.match_projective(*tgt_frame, frame->map_points);
//            for (auto &match : matches) {
//                if (tgt_frame->map_points[match.first] == nullptr && frame->map_points[match.second] != nullptr) {
//                    tgt_frame->map_points[match.first] = frame->map_points[match.second];
//                    add_num ++;
//                }
//            }
//        }
//        std::cout<<" add num: " << add_num << std::endl;
//
//        BundleAdjustment ba;
//        std::vector<bool> inliers = ba.optimize_pose(tgt_frame);
////        std::cout<<"T " << tgt_frame->get_T() << std::endl;;
//        int inliers_num = 0;
//        for (int j = 0;j < inliers.size();j ++) {
//            if (!inliers[j]) {
//                tgt_frame->map_points[j] = nullptr;
//            } else {
//                inliers_num ++;
//            }
//        }
//        std::cout<<" end inliers: " << inliers_num << std::endl;
        init_T = tgt_frame->get_T();
        vis.add_current_frame(tgt_frame);

//        std::this_thread::sleep_for(std::chrono::milliseconds(100));


        cv::Mat show = tgt_frame->image.clone();
        if (show.channels() == 1) {
            cv::cvtColor(show, show, cv::COLOR_GRAY2BGR);
        }

        for (int i = 0;i < tgt_frame->map_points.size(); i++) {
            const auto& mp = tgt_frame->map_points[i];
            if (mp != nullptr) {
                Eigen::Vector3d P = mp->x3D;
                Eigen::Vector3d P_ = tgt_frame->get_R().transpose() * (P - tgt_frame->get_t());
                Eigen::Vector2d uv = tgt_frame->get_camera()->project(P_);
                double error = (uv - tgt_frame->get_features_uv()[i]).norm();
                cv::circle(show, cv::Point((int)uv[0], (int)uv[1]), 4, cv::Scalar(0, 255, 0), 2);
                cv::circle(show, cv::Point((int)tgt_frame->get_features_uv()[i][0], (int)tgt_frame->get_features_uv()[i][1]), 2, cv::Scalar(0, 0, 255), 2);
            }
        }
        cv::imshow("image", show);
        cv::waitKey(10);
    }

    vis_thread.join();







//    // 4. find more connected frames
//    std::vector<Frame*> connected_frames_1, connected_frames_2;
//    connected_frames_1 = frames[close_frame_ids[0]].get_connected_frames();
//    for (auto & i : connected_frames_1) {
//        connected_frames_2.push_back(i);
//        std::vector<Frame*> connected_frames = i->get_connected_frames();
//        for (auto & connected_frame : connected_frames) {
//            bool is_in = false;
//            for (auto & j : connected_frames_1) {
//                if (connected_frame->id == j->id) {
//                    is_in = true;
//                    break;
//                }
//            }
//            if (!is_in) {
//                connected_frames_2.push_back(connected_frame);
//            }
//        }
//    }
//    // 5. projection match
//    for (auto & frame : connected_frames_2) {
//        std::vector<std::pair<int, int>> matches = matcher.match_re_projective(&tgt_frame, frame);
//        for (auto &match : matches) {
//            if (frame->map_points[match.second] != nullptr) {
//                tgt_frame->map_points[match.first] = frame->map_points[match.second];
//            }
//        }
//    }
//    // 6. PNP solve the pose
//    BundleAdjustment ba;
//    ba.optimize_pose(&tgt_frame);



}