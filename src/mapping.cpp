#include "mapping.h"
#include "matcher.h"
#include "triangulation.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <unordered_map>
#include "bundle_adjustment.h"


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

        std::cout << "step 1: Constructing initial map points for stereo !" << std::endl;
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
            frame_cnt ++;
        }
        std::cout << "Initial constructed " << map_point_cnt << " map points" << std::endl;

        std::cout << "step 2: Find the connected frames !" << std::endl;
        std::cout << "step 2.1: Find the connected frames by re-projecting the map points to other frames !" << std::endl;
        std::vector<std::vector<std::shared_ptr<Frame>>> connected_frames;
        connected_frames.resize(frames.size());
        for (int i = 0; i < frames.size(); i++) {
            std::shared_ptr<Frame> frame0 = frames[i];
            for (int j = i + 1; j < frames.size(); j ++) {
                std::shared_ptr<Frame> frame1 = frames[j];
                double dist = (frame0->get_t() - frame1->get_t()).norm();
                if (dist > 2) {
                    continue;
                }
                int project_success = 0;
                for (const auto& mp : frame0->map_points) {
                    if (mp != nullptr) {
                        Eigen::Vector3d P = mp->x3D;
                        Eigen::Vector3d P_ = frame1->get_R().transpose() * (P - frame1->get_t());
                        Eigen::Vector2d uv = frame1->get_camera()->project(P_);
                        if (frame1->in_image(uv)) {
                            project_success++;
                        }
                        if (project_success > 10) {
                            connected_frames[i].push_back(frame1);
                            break;
                        }
                    }
                }
            }
        }

        std::cout<<"step 2.2:  Find map point connections: "<<std::endl;
        Matcher matcher;

        // init the connected frames
        map.connected_frames.resize(frames.size());
        for (int i = 0;i < frames.size(); i++) {
            map.connected_frames[i].emplace_back(frames[i]);
        }

        // match the map points
        for (int i = 0;i < frames.size(); i++) {
            std::shared_ptr<Frame> frame0 = frames[i];
            std::vector<std::shared_ptr<Frame>> frames1 = connected_frames[i];

            for (const auto& frame1 : frames1) {
                std::vector<std::pair<int, int>> matches = matcher.match_re_projective(frame0, frame1);

                if (matches.size() > 30){// connect frames
                    frame0->add_linked_frame(frame1);
                    frame1->add_linked_frame(frame0);
                    map.connected_frames[i].emplace_back(frame1);
                    map.connected_frames[frame1->id].emplace_back(frame0);
                }

                for (const auto& match : matches) {// connect map points
                    const auto& mp0 = frame0->map_points[match.first];
                    const auto& mp1 = frame1->map_points[match.second];
                    mp0->add_measurement(frame1, match.second);
                    if (mp1 != nullptr) {
                        mp1->add_measurement(frame0, match.first);
                    }
                }
            }
        }
    }

    void Mapping::refine_map() {
        std::cout << "Refine map step 1: Update all map points !" << std::endl;
        std::cout<<" map points size: "<<map.map_points_count<<std::endl;
        BundleAdjustment ba;
        for (const auto& frame : map.frames_) {
            for (int mp_id = 0; mp_id < frame->map_points.size(); mp_id ++) {
                const auto& mp = frame->map_points[mp_id];
                if (mp != nullptr) {
                    // find all connected map points
                    std::vector<std::shared_ptr<Frame>> c_frames;
                    std::vector<int> c_ids;
                    std::unordered_map<std::shared_ptr<Frame>, int> connected_mps;
                    for (int i = 0; i < mp->frames.size(); i++) {
                        if (connected_mps.find(mp->frames[i]) == connected_mps.end()) {
                            connected_mps[mp->frames[i]] = mp->frame_feature_ids[i];
                            std::shared_ptr<MapPoint> c_mp = mp->frames[i]->map_points[mp->frame_feature_ids[i]];
                            if (c_mp != nullptr) {
                                for (int j = 0; j < c_mp->frames.size(); j++) {
                                    if (connected_mps.find(c_mp->frames[j]) == connected_mps.end()) {
                                        connected_mps[c_mp->frames[j]] = c_mp->frame_feature_ids[j];
                                    }
                                }
                            }
                        }
                    }
                    // add current frame
                    c_frames.push_back(frame);
                    c_ids.push_back(mp_id);

                    for (const auto& kv : connected_mps) {
                        if (kv.first == frame) {
                            continue;
                        }
                        c_frames.push_back(kv.first);
                        c_ids.push_back(kv.second);
                    }
                    // optimize the map point position
                    if (c_frames.size() > 1) {
                        // visualize the connected frames and project the map point
//                        {
//                            Eigen::Vector3d xyz = mp->x3D;
//                            for (int i = 0; i < c_frames.size(); i++){
//                                cv::Mat show = c_frames[i]->image.clone();
//                                if (show.channels() == 1) {
//                                    cv::cvtColor(show, show, cv::COLOR_GRAY2BGR);
//                                }
//                                cv::circle(show, cv::Point((int)c_frames[i]->get_features_uv()[c_ids[i]][0], (int)c_frames[i]->get_features_uv()[c_ids[i]][1]), 4, cv::Scalar(0, 0, 255), 2);
//                                Eigen::Vector2d uv = c_frames[i]->get_camera()->project(c_frames[i]->get_R().transpose() * (xyz - c_frames[i]->get_t()));
//                                cv::circle(show, cv::Point((int)uv[0], (int)uv[1]), 2, cv::Scalar(0, 255, 0), 2);
//                                cv::imwrite("show" + std::to_string(i) + ".png", show);
//                            }
//                        }
//                        double error = 0, update_error = 0;
                        Eigen::Vector3d xyz = mp->x3D;
//                        for (int i = 0; i < c_frames.size(); i++){
//                            Eigen::Vector2d uv = c_frames[i]->get_camera()->project(c_frames[i]->get_R().transpose() * (xyz - c_frames[i]->get_t()));
//                            error += (uv - c_frames[i]->get_features_uv()[c_ids[i]]).norm();
//                        }
                        std::vector<int> status(c_frames.size(), 1);
                        ba.optimize_structure(c_frames, c_ids, xyz, status);
                        // update the map point position
                        mp->x3D = xyz;
//                        for (int i = 0; i < c_frames.size(); i++){
//                            Eigen::Vector2d uv = c_frames[i]->get_camera()->project(c_frames[i]->get_R().transpose() * (xyz - c_frames[i]->get_t()));
//                            double e = (uv - c_frames[i]->get_features_uv()[c_ids[i]]).norm();
//                            update_error += e;
//                        }
//                        std::cout << "error: " << error << " -> " << update_error << std::endl;

                        // visualize the connected frames and project the map point
//                        {
//                            for (int i = 0; i < c_frames.size(); i++){
//                                cv::Mat show = c_frames[i]->image.clone();
//                                if (show.channels() == 1) {
//                                    cv::cvtColor(show, show, cv::COLOR_GRAY2BGR);
//                                }
//                                cv::circle(show, cv::Point((int)c_frames[i]->get_features_uv()[c_ids[i]][0], (int)c_frames[i]->get_features_uv()[c_ids[i]][1]), 4, cv::Scalar(0, 0, 255), 2);
//                                Eigen::Vector2d uv = c_frames[i]->get_camera()->project(c_frames[i]->get_R().transpose() * (xyz - c_frames[i]->get_t()));
//                                cv::circle(show, cv::Point((int)uv[0], (int)uv[1]), 2, cv::Scalar(0, 255, 0), 2);
//                                cv::imwrite("u_show" + std::to_string(i) + ".png", show);
//                            }
//                        }

                        // fuse the map points
                        for (int i = 0; i < c_frames.size(); i++) {
                            if (status[i] == 1 && c_frames[i]->map_points[c_ids[i]] != mp) {
                                if (c_frames[i]->map_points[c_ids[i]] != nullptr) {
                                    map.remove_map_point(c_frames[i]->map_points[c_ids[i]]->id);
                                }
                                c_frames[i]->map_points[c_ids[i]] = mp;
                            }
                        }

                    }
                }
            }
        }
        std::cout<<" map points size: "<<map.map_points_count<<std::endl;

        std::cout << "Refine map step 2: update all frame pose !" << std::endl;
//        for (const auto& frame : map.frames_) {
//            ba.optimize_pose(frame);
//        }

        std::cout << "Refine map step 3: full bundle adjustment !" << std::endl;

        /**
            // try linking to other frames
            Eigen::Vector3d current_t = frames[frame_cnt]->get_t();
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
