#include "mapping.h"
#include "matcher.h"
#include "triangulation.h"
#include <iostream>
#include <opencv2/opencv.hpp>

namespace VISUAL_MAPPING {

    /**
     * @brief Construct the initial map
     *  1. Construct the initial map points from stereo images
     *  2. Triangulate the map points
     *  3. Link the map points to other frames
     * @param frames
     */
    void Mapping::construct_initial_map(std::vector<std::shared_ptr<Frame>>& frames) {
        map.frames_ = frames;
        int map_point_cnt = 0; // map point id
        int frame_cnt = 0; // frame id
        Matcher matcher;
        Triangulation triangulation;

        for (auto& frame : frames) {// for each frame in the sequence
            for (int i = 0; i < frame->get_features_uv().size(); i++) {
                Eigen::Vector2d uv = frame->get_features_uv()[i];
                double depth = frame->get_features_depth()[i];
                double cov = frame->features_depth_cov[i];
                std::shared_ptr<MapPoint> mp_ptr = frame->map_points[i];
                // if the depth is valid and the map point is not created
                if (depth > 0.01 && mp_ptr == nullptr) {
                    // create a new map point and add it to the map
                    Eigen::Vector3d x3D = frame->get_camera()->pixel2camera(uv, depth);
                    x3D = frame->get_R() * x3D + frame->get_t();
                    std::shared_ptr<MapPoint> map_point = std::make_shared<MapPoint>(uv, x3D, cov, frame->id, i, map_point_cnt);
                    frame->map_points[i] = map_point;
                    map.add_map_point(map_point);
                    map_point_cnt++;
                }
            }

            // try linking to other frames
            Eigen::Vector3d current_t = frames[frame_cnt]->get_t();
            /**
            for (int j = frame_cnt+1; j < frames.size(); j++) {
                Eigen::Vector3d frame_t  = frames[j]->get_t();
                double dist = (current_t - frame_t).norm();
                // TODO set the threshold to be a parameter
                if (dist < 2) {
                    // 1. try matching the point to the frame
                    std::vector<std::pair<int, int>> matches = matcher.match_re_projective(frames[frame_cnt], frames[j]);
                    // 2. compute depth
                    std::vector<double> prev_error;
                    std::vector<double> update_error;
                    for (auto& match : matches) {
                        // re projection error
                        {
                            Eigen::Vector3d P = frames[frame_cnt]->map_points[match.first]->x3D;
                            Eigen::Vector3d P_ = frames[j]->get_R().transpose() * (P - frames[j]->get_t());
                            Eigen::Vector2d uv = frames[j]->get_camera()->project(P_);
                            double error = (uv - frames[j]->get_features_uv()[match.second]).norm();
                            prev_error.push_back(error);
                        }

                        // choose the point with the smallest covariance
                        frames[frame_cnt]->map_points[match.first]->add_frame(
                                frames[j]->id,
                                match.second,
                                frames[j]->features_depth_cov[match.second],
                                frames[j]->get_features_uv()[match.second],
                                frames[j]->get_R() * frames[j]->get_camera()->pixel2camera(frames[j]->get_features_uv()[match.second],
                                                                                         frames[j]->get_features_depth()[match.second]) + frames[j]->get_t());

                        frames[j]->map_points[match.second] = frames[frame_cnt]->map_points[match.first];

                        Eigen::Matrix4d T12 = frames[frame_cnt]->get_T().inverse() * frames[j]->get_T();
                        frames[frame_cnt]->get_features_uv()[match.first];
                        std::pair<double, double> depth= triangulation.triangulate(frames[frame_cnt]->get_camera(),
                                                                                   frames[j]->get_camera(),
                                                                                   frames[frame_cnt]->get_features_uv()[match.first],
                                                                                   frames[j]->get_features_uv()[match.second],
                                                                                   T12.block<3, 3>(0, 0),
                                                                                   T12.block<3, 1>(0, 3));
                        if (depth.first > 0) {
                            if (depth.second > 0 && depth.second < frames[frame_cnt]->map_points[match.first]->min_cov)
                            {
                                frames[frame_cnt]->map_points[match.first]->min_cov = depth.second;
                                Eigen::Vector3d P1 = frames[frame_cnt]->get_camera()->pixel2camera(frames[frame_cnt]->get_features_uv()[match.first], depth.first);
                                Eigen::Vector3d x3D = frames[frame_cnt]->get_R() * P1 + frames[frame_cnt]->get_t();

                                // check error
                                Eigen::Vector3d P = frames[frame_cnt]->map_points[match.first]->x3D;
                                Eigen::Vector3d P_ = frames[j]->get_R().transpose() * (P - frames[j]->get_t());
                                Eigen::Vector2d uv = frames[j]->get_camera()->project(P_);
                                double error1 = (uv - frames[j]->get_features_uv()[match.second]).norm();

                                P_ = frames[j]->get_R().transpose() * (x3D - frames[j]->get_t());
                                uv = frames[j]->get_camera()->project(P_);
                                double error2 = (uv - frames[j]->get_features_uv()[match.second]).norm();

                                if (error1 > error2) {
                                    frames[frame_cnt]->map_points[match.first]->x3D = x3D;
                                }
                            }
                        }

                        // re projection error
                        {
                            Eigen::Vector3d P = frames[frame_cnt]->map_points[match.first]->x3D;
                            Eigen::Vector3d P_ = frames[j]->get_R().transpose() * (P - frames[j]->get_t());
                            Eigen::Vector2d uv = frames[j]->get_camera()->project(P_);
                            double error = (uv - frames[j]->get_features_uv()[match.second]).norm();
                            update_error.push_back(error);
                        }
                    }
                }
            }
            **/
            frame_cnt ++;
        }
        std::cout << "Constructed " << map_point_cnt << " map points" << std::endl;
    }

    void Mapping::refine_map() {

//                    std::cout<<" re projection error: "<<std::endl;
//                    for (int i = 0; i < prev_error.size(); i++) {
//                        std::cout << prev_error[i] << " -> " << update_error[i] << std::endl;
//                    }
//                    for (auto& match : matches){
//                        cv::Mat show = frames[j].image.clone();
//                        cv::cvtColor(show, show, cv::COLOR_GRAY2BGR);
//
//                        for (int i = 0;i < 1000;i ++) {
//                            Eigen::Vector3d P1_ = frame.get_camera()->pixel2camera(frame.get_features_uv()[match.first], frame.get_features_depth()[match.first] + i * 0.3);
//                            Eigen::Vector3d P_ = frame.get_R() * P1_ + frame.get_t();
//                            Eigen::Vector3d P2 = frames[j].get_R().transpose() * (P_ - frames[j].get_t());
//                            Eigen::Vector2d uv = frames[j].get_camera()->project(P2);
////                            std::cout<<"R1 "<<frame.get_R()<<std::endl;
////                            std::cout<<"t1 "<<frame.get_t().transpose()<<std::endl;
////                            std::cout<<"R2 "<<frames[j].get_R()<<std::endl;
////                            std::cout<<"t2 "<<frames[j].get_t().transpose()<<std::endl;
////                            std::cout<<"uv: "<<uv.transpose()<<std::endl;
//                            cv::circle(show, cv::Point((int)uv[0], (int)uv[1]), 2, cv::Scalar(0, 255, 0), 2);
//                        }
//                        cv::circle(show, cv::Point((int)frames[j].get_features_uv()[match.second][0], (int)frames[j].get_features_uv()[match.second][1]), 2, cv::Scalar(0, 0, 255), 2);
//                        cv::imshow("show", show);
//
//                        cv::Mat show2 = frame.image.clone();
//                        cv::cvtColor(show2, show2, cv::COLOR_GRAY2BGR);
//                        cv::circle(show2, cv::Point((int)frames[frame_cnt].get_features_uv()[match.first][0], (int)frames[frame_cnt].get_features_uv()[match.first][1]), 2, cv::Scalar(0, 0, 255), 2);
//                        cv::imshow("show2", show2);
//
//                        cv::waitKey(0);
//                    }

//        for (auto& frame : frames) {
//            int valid_mp = 0;
//            for (auto & map_point : frame.map_points) {
//                if (map_point != nullptr && map_point->frame_ids.size() > 1) {
//                    valid_mp ++;
//                }
//            }
//            std::cout<<"frame_cnt: "<<frame_cnt<<" valid map points: "<<valid_mp;
//            if (valid_mp > 80) {
//                int delete_mp = valid_mp - 80;
//                for (auto & map_point : frame.map_points) {
//                    if ((map_point != nullptr && map_point->frame_ids.size() == 1) || (map_point != nullptr && map_point->frame_ids.size() > 1 && delete_mp > 0)) {
//                        if (map_point->frame_ids.size() > 1) {
//                            delete_mp --;
//                        }
//                        map.remove_map_point(map_point->id);
//                        if (map_point != nullptr) {
//                            int size = (int)map_point->frame_ids.size();
//                            for (int k = 0;k < size; k++) {
//                                int frame_id = map_point->frame_ids[k];
//                                int feature_id = map_point->feature_ids[k];
//                                frames[frame_id].map_points[feature_id] = nullptr;
//                                if (map_point == nullptr) {
//                                    break;
//                                }
//                            }
//                        }
//                        map_point = nullptr;
//                        cnt --;
//                    }
//                }
//                std::cout<<" get "<<50<<" map points"<<std::endl;
//            } else {
//                // try to remove the map point
//                int target = 80 - valid_mp;
//                for (auto & map_point : frame.map_points) {
//                    if (map_point != nullptr && map_point->frame_ids.size() == 1) {
//                        if (target == 0) {
//                            map.remove_map_point(map_point->id);
//                            if (map_point != nullptr) {
//                                int size = (int)map_point->frame_ids.size();
//                                for (int k = 0;k < size; k++) {
//                                    int frame_id = map_point->frame_ids[k];
//                                    int feature_id = map_point->feature_ids[k];
//                                    frames[frame_id].map_points[feature_id] = nullptr;
//                                }
//                            }
//                            map_point = nullptr;
//                            cnt --;
//                            break;
//                        } else {
//                            target --;
//                        }
//                    }
//                }
//                std::cout<<" get "<<(50 - target)<<" map points"<<std::endl;
//            }
//        }
    }

}
