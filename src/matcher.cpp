#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>
#include "matcher.h"

namespace VISUAL_MAPPING {
    void Matcher::match_bf(Frame *frame1, Frame *frame2,
                           std::vector<cv::DMatch> &matches) {

    }

    void Matcher::match_epipolar(Frame *frame1, Frame *frame2,
                                               std::vector<cv::DMatch> &matches) {

    }

    std::vector<std::pair<int, int>> Matcher::match_re_projective(Frame *frame1, Frame *frame2) {
        std::vector<std::pair<int, int>> matches;
        for (int i = 0;i < frame1->map_points.size(); i++) {
            if (frame1->map_points[i] != nullptr) {
                // 1. project the point to the second frame
                Eigen::Vector3d x3D = frame1->map_points[i]->x3D;
                Eigen::Vector3d x3D_2 = frame2->get_R().transpose() * (x3D - frame2->get_t());
                Eigen::Vector2d uv = frame2->get_camera()->project(x3D_2);
//                std::cout<<"features_uv: "<<frame1->get_features_uv()[i].transpose()<<std::endl;
//                std::cout << "uv: " << uv.transpose() << std::endl;
                // 2. get around features
                std::vector<int> around_ids = frame2->get_around_features(uv, 50);
                // 3. match the features
                cv::Mat desc_1 = frame1->descriptors.row(i);
                double min_dist = 1000;
                int min_id = -1;
                for (int j = 0; j < around_ids.size();j ++) {
                    cv::Mat desc_2 = frame2->descriptors.row(around_ids[j]);
                    double dist = cv::norm(desc_1, desc_2);
                    if (dist < min_dist) {
                        min_dist = dist;
                        min_id = around_ids[j];
                    }
                }
                if (min_id != -1 && min_dist < 100) {
//                    if (frame2->features_depth[min_id] > 0)
                    {
                        // 4. check the reprojection error
//                        Eigen::Vector3d x3D_ = frame2->map_points[min_id]->x3D;
//                        Eigen::Vector3d x3D_1 = frame1->get_R().transpose() * (x3D_2 - frame1->get_t());
//                        Eigen::Vector2d uv_1 = frame1->get_camera()->project(x3D_1);
//                        double error = (uv_1 - frame1->get_features_uv()[i]).norm();
//                        if (error < 10) {
                            matches.emplace_back(i, min_id);
//                        }
                    }
                }

//                cv::Mat img1 = frame1->image.clone();
//                cv::Mat img2 = frame2->image.clone();
//                cv::cvtColor(img1, img1, cv::COLOR_GRAY2BGR);
//                cv::cvtColor(img2, img2, cv::COLOR_GRAY2BGR);
//                cv::circle(img1, cv::Point((int)frame1->get_features_uv()[i].x(), (int)frame1->get_features_uv()[i].y()), 2, cv::Scalar(0, 255, 0), 2);
//
//                for (int i = 0; i < frame2->get_features_uv().size(); i++) {
//                    if (frame2->features_depth[i] > 0) {
//                        cv::circle(img2, cv::Point((int)frame2->get_features_uv()[i].x(), (int)frame2->get_features_uv()[i].y()), 3, cv::Scalar(0, 255, 0), 3);
//                    }
//                }
//                // around_ids
//                for (int around_id : around_ids) {
//                    if (frame2->features_depth[around_id] > 0)
//                    {
//                        cv::circle(img2, cv::Point((int) frame2->get_features_uv()[around_id].x(),
//                                                   (int) frame2->get_features_uv()[around_id].y()), 2,
//                                   cv::Scalar(0, 0, 255), 2);
//                    }
//                }
//                if (min_id != -1 && min_dist < 100) {
//                    cv::circle(img2, cv::Point((int)frame2->get_features_uv()[min_id].x(), (int)frame2->get_features_uv()[min_id].y()), 3, cv::Scalar(0, 0, 255), 3);
//                }
//                cv::circle(img2, cv::Point((int)uv.x(), (int)uv.y()), 1, cv::Scalar(255, 0, 0), 2);
//
//                cv::Mat show = cv::Mat(std::max(img1.rows, img2.rows), img1.cols + img2.cols, CV_8UC3);
//                img1.copyTo(show(cv::Rect(0, 0, img1.cols, img1.rows)));
//                img2.copyTo(show(cv::Rect(img1.cols, 0, img2.cols, img2.rows)));
//                if (min_id != -1 && min_dist < 100)
//                {
//                    if (frame2->features_depth[min_id] > 0) {
//                        cv::line(show, cv::Point((int)frame1->get_features_uv()[i].x(), (int)frame1->get_features_uv()[i].y()),
//                                 cv::Point((int)frame2->get_features_uv()[min_id].x() + img1.cols, (int)frame2->get_features_uv()[min_id].y()),
//                                 cv::Scalar(0, 0, 255), 2);
//                    }
//                }
//                cv::imshow("show", show);
//                cv::waitKey(0);
            }
        }

        // draw matches
//        cv::Mat img1 = frame1->image.clone();
//        cv::Mat img2 = frame2->image.clone();
//        cv::cvtColor(img1, img1, cv::COLOR_GRAY2BGR);
//        cv::cvtColor(img2, img2, cv::COLOR_GRAY2BGR);
//
//        for (int i = 0; i < frame1->get_features_uv().size(); i++) {
//            if (frame1->features_depth[i] > 0) {
//                cv::circle(img1, cv::Point((int)frame1->get_features_uv()[i].x(), (int)frame1->get_features_uv()[i].y()), 2, cv::Scalar(0, 255, 0), 2);
//            }
//        }
//        for (int i = 0; i < frame2->get_features_uv().size(); i++) {
//            if (frame2->features_depth[i] > 0) {
//                cv::circle(img2, cv::Point((int)frame2->get_features_uv()[i].x(), (int)frame2->get_features_uv()[i].y()), 2, cv::Scalar(0, 255, 0), 2);
//            }
//        }
////        cv::imshow("img1", img1);
////        cv::imshow("img2", img2);
//
//        cv::Mat show = cv::Mat(std::max(img1.rows, img2.rows), img1.cols + img2.cols, CV_8UC3);
//        img1.copyTo(show(cv::Rect(0, 0, img1.cols, img1.rows)));
//        img2.copyTo(show(cv::Rect(img1.cols, 0, img2.cols, img2.rows)));
//
//        for (auto match : matches) {
//            cv::line(show, cv::Point((int)frame1->get_features_uv()[match.first].x(), (int)frame1->get_features_uv()[match.first].y()),
//                     cv::Point((int)frame2->get_features_uv()[match.second].x() + img1.cols, (int)frame2->get_features_uv()[match.second].y()),
//                     cv::Scalar(0, 0, 255), 2);
//        }
//        cv::imshow("show", show);
//        cv::waitKey(0);

        return matches;
    }

    std::vector<std::pair<int, int>> Matcher::match_stereo(
            cv::Mat img1, cv::Mat img2,
            std::vector<Eigen::Vector2d> &features_uv_left,
            std::vector<Eigen::Vector2d> &features_uv_right,
            cv::Mat &descriptors_left, cv::Mat &descriptors_right) {
        std::vector<std::pair<int, int>> matches;
        // 1. brute force matching
        cv::BFMatcher matcher(cv::NORM_L2);
        std::vector<std::vector<cv::DMatch>> knn_matches;
        std::vector<cv::DMatch> good_matches;
        matcher.knnMatch(descriptors_left, descriptors_right, knn_matches, 2);
        // 2. filter matches
        for (int i = 0; i < knn_matches.size(); i++) {
            if (knn_matches[i][0].distance < 0.8 * knn_matches[i][1].distance) {
                matches.emplace_back(i, knn_matches[i][0].trainIdx);
                good_matches.push_back(knn_matches[i][0]);
            }
        }

//    std::vector<cv::KeyPoint> features_left, features_right;
//    for (auto &uv : features_uv_left) {
//        cv::KeyPoint kp((float)uv.x(), (float)uv.y(), 1.0f, -1, 0, 0, -1);
//        features_left.push_back(kp);
//    }
//    for (auto &uv : features_uv_right) {
//        cv::KeyPoint kp((float)uv.x(), (float)uv.y(), 1.0f, -1, 0, 0, -1);
//        features_right.push_back(kp);
//    }
//    cv::Mat show;
//    cv::drawMatches(img1, features_left, img2, features_right, good_matches, show);
//    cv::imshow("Matches", show);
//    cv::waitKey(1000);

        return matches;
    }

    std::vector<std::pair<int, int>> Matcher::match_knn(Frame& frame1, Frame& frame2) {
        std::vector<std::pair<int, int>> matches;
        // 1. brute force matching
        cv::BFMatcher matcher(cv::NORM_L2);
        std::vector<std::vector<cv::DMatch>> knn_matches;
        std::vector<cv::DMatch> good_matches;
        matcher.knnMatch(frame1.descriptors, frame2.descriptors, knn_matches, 2);
        // 2. filter matches
        for (int i = 0; i < knn_matches.size(); i++) {
            if (knn_matches[i][0].distance < 0.8 * knn_matches[i][1].distance) {
                matches.emplace_back(i, knn_matches[i][0].trainIdx);
                good_matches.push_back(knn_matches[i][0]);
            }
        }
        return matches;
    }

    std::vector<std::pair<int, int>>
    Matcher::match_projective(Frame &frame, std::vector<std::shared_ptr<MapPoint>> map_points) {
        std::vector<std::pair<int, int>> matches;
        for (int i = 0;i < map_points.size();i ++) {
            std::shared_ptr<MapPoint> mp = map_points[i];
            if (mp != nullptr) {
                Eigen::Vector3d x3D = mp->x3D;
                Eigen::Vector3d x3D_2 = frame.get_R().transpose() * (x3D - frame.get_t());
                Eigen::Vector2d uv = frame.get_camera()->project(x3D_2);
                std::vector<int> around_ids = frame.get_around_features(uv, 50);
                cv::Mat desc_1 = mp->descriptor;
                double min_dist = 1000;
                int min_id = -1;
                for (int around_id : around_ids) {
                    cv::Mat desc_2 = frame.descriptors.row(around_id);
                    double dist = cv::norm(desc_1, desc_2);
                    if (dist < min_dist) {
                        min_dist = dist;
                        min_id = around_id;
                    }
                }
                if (min_id != -1 && min_dist < 100) {
                    matches.emplace_back(min_id, i);
                }
            }
        }
        return matches;
    }
}

