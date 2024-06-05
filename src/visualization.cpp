#include "visualization.h"
#include <pangolin/pangolin.h>
#include "map_point.h"

namespace VISUAL_MAPPING {

    void Visualization::draw_matches(Frame *frame1, Frame *frame2, std::vector<cv::DMatch> &matches) {

    }

    void Visualization::draw_keypoint(Frame *frame) {

    }

    void Visualization::draw_camera_poses(std::vector<Frame> *frames) {

    }

    void Visualization::draw_ba_map(std::vector<Frame> *frames) {

    }

    void Visualization::run(Visualization* visualization,
                            const std::vector<Frame> &frames,
                            const Map& map) {
        std::cout << "Running visualization" << std::endl;
        pangolin::CreateWindowAndBind("Map Viewer",1024,768);
        glEnable(GL_DEPTH_TEST);

        // Issue specific OpenGl we might need
        glEnable (GL_BLEND);
        glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);


        // Define Projection and initial ModelView matrix
        pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(768,768,420,420,320,240,0.2,100),
                pangolin::ModelViewLookAt(-2,2,-2, 0,0,0, pangolin::AxisY)
        );

        // Create Interactive View in window
        pangolin::View& d_cam = pangolin::CreateDisplay()
                .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
                .SetHandler(new pangolin::Handler3D(s_cam));


        while( !pangolin::ShouldQuit() )
        {
            // Clear screen and activate view to render into
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            d_cam.Activate(s_cam);
            glClearColor(1.0f,1.0f,1.0f,1.0f);

            Frame last_frame = frames[0];
//            for (auto& frame : frames) {
//                Eigen::Matrix3d R = frame.get_R();
//                Eigen::Vector3d t = frame.get_t();
//
//                Eigen::Matrix3d R_S = R * 0.1;
//                glLineWidth(2);
//                glColor3f(0.0f,1.0f,0.0f);
//                glBegin(GL_LINES);
//                glVertex3f((float)t[0],(float)t[1],(float)t[2]);
//                glVertex3f((float)(t[0]+R_S(0,0)),(float)(t[1]+R_S(1,0)),(float)(t[2]+R_S(2,0)));
//                glVertex3f((float)t[0],(float)t[1],(float)t[2]);
//                glVertex3f((float)(t[0]+R_S(0,1)),(float)(t[1]+R_S(1,1)),(float)(t[2]+R_S(2,1)));
//                glVertex3f((float)t[0],(float)t[1],(float)t[2]);
//                glVertex3f((float)(t[0]+R_S(0,2)),(float)(t[1]+R_S(1,2)),(float)(t[2]+R_S(2,2)));
//                glEnd();
//
//                {
//                    Eigen::Vector3d t_last = last_frame.get_t();
//                    glLineWidth(2);
//                    glColor3f(0.0f,0.0f,1.0f);
//                    glBegin(GL_LINES);
//                    glVertex3f((float)t[0],(float)t[1],(float)t[2]);
//                    glVertex3f((float)t_last[0],(float)t_last[1],(float)t_last[2]);
//                    glEnd();
//                }

                // Draw map point
//                {
//                    static int cnt = 0;
//
//                    auto feature_uv = frame.get_features_uv();
//                    auto feature_depth = frame.get_features_depth();
//                    for (int i = 0; i < feature_uv.size(); i++) {
//                        Eigen::Vector2d uv = feature_uv[i];
//                        double depth = feature_depth[i];
//                        if (depth > 0.01) {
//                            Eigen::Vector3d p = frame.get_camera()->pixel2camera(uv, depth);
//                            p = R * p + t;
//                            glPointSize(2);
//                            if (frame.map_points[i] != nullptr && frame.map_points[i]->frame_ids.size() > 1) {
//                                glColor3f(1.0f,0.0f,0.0f);
//                            } else if (frame.map_points[i] != nullptr){
//                                glColor3f(0.0f,1.0f,0.0f);
//                            } else{
//                                glColor3f(0.0f,0.0f,0.0f);
//                            }
//
//                            cnt ++;
//                            glBegin(GL_POINTS);
//                            glVertex3f((float)p[0],(float)p[1],(float)p[2]);
//                            glEnd();
//                        }
//                    }
//                }

//                last_frame = frame;
//            }

            // map
            for (int i = 0;i < map.max_id; i++) {
                if (map.map_points.find(i) == map.map_points.end()) {
                    continue;
                }
                auto map_point = map.map_points.at(i);
                if (map_point != nullptr) {
                    Eigen::Vector3d p = map_point->x3D;
//                    std::cout<<"p: "<<p.transpose()<<std::endl;
                    glPointSize(2);
                    if (map_point->frame_ids.size() > 1) {
                        glColor3f(1.0f,0.0f,0.0f);
                    } else {
                        glColor3f(0.0f,1.0f,0.0f);
                    }

                    glBegin(GL_POINTS);
                    glVertex3f((float)p[0],(float)p[1],(float)p[2]);
                    glEnd();
                }
            }

            // current frame
            if (visualization->current_frame != nullptr) {
                // Draw camera pose
                Eigen::Matrix3d R = visualization->current_frame->get_R();
                Eigen::Vector3d t = visualization->current_frame->get_t();
                Eigen::Matrix3d R_S = R * 0.1;
                glLineWidth(2);
                glColor3f(0.0f,1.0f,0.0f);
                glBegin(GL_LINES);
                glVertex3f((float)t[0],(float)t[1],(float)t[2]);
                glVertex3f((float)(t[0]+R_S(0,0)),(float)(t[1]+R_S(1,0)),(float)(t[2]+R_S(2,0)));
                glVertex3f((float)t[0],(float)t[1],(float)t[2]);
                glVertex3f((float)(t[0]+R_S(0,1)),(float)(t[1]+R_S(1,1)),(float)(t[2]+R_S(2,1)));
                glVertex3f((float)t[0],(float)t[1],(float)t[2]);
                glVertex3f((float)(t[0]+R_S(0,2)),(float)(t[1]+R_S(1,2)),(float)(t[2]+R_S(2,2)));
                glEnd();

                // draw connected map points
                for (const auto &mp : visualization->current_frame->map_points) {
                    if (mp != nullptr) {
                        int id = mp->id;
                        if (map.map_points.find(id) != map.map_points.end()) {
                            auto map_point = map.map_points.at(id);
                            Eigen::Vector3d p = map_point->x3D;
                            if (p.z() < 0)
                                continue;
                            glLineWidth(1);
                            glColor3f(0.5f,0.5f,0.9f);

                            glBegin(GL_LINES);
                            glVertex3f((float)t[0],(float)t[1],(float)t[2]);
                            glVertex3f((float)p[0],(float)p[1],(float)p[2]);
                            glEnd();
                        }
                    }
                }
            }

            // Swap frames and Process Events
            pangolin::FinishFrame();
        }
    }
}

