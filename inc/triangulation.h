#ifndef __TRIANGULATION_H_
#define __TRIANGULATION_H_
#include "camera.h"
#include "frame.h"
#include "Eigen/Dense"

namespace VISUAL_MAPPING {

    class Triangulation {
    public:
        Triangulation() = default;
        void triangulate(Frame* frame1, Frame* frame2, std::vector<cv::DMatch>& matches);
        std::pair<double, double> triangulate(Camera* camera1, Camera *camera2, const Eigen::Vector2d& kp1, const Eigen::Vector2d& kp2, const Eigen::Matrix3d& R, const Eigen::Vector3d& t);
        std::vector<std::pair<double, double>> multiview_triangulate(std::shared_ptr<Frame> frame, std::vector<int> ids, std::vector<std::shared_ptr<Frame>> c_frames, std::vector<std::vector<int>> matches);
    };

}


#endif //__TRIANGULATION_H_
