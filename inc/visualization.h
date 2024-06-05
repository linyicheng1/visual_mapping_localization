#ifndef __VISUALIZATION_H_
#define __VISUALIZATION_H_
#include "frame.h"
#include "opencv2/core.hpp"
//#include "pangolin/pangolin.h"

namespace VISUAL_MAPPING {

    class Visualization {
    public:
        Visualization() = default;
        static void run(Visualization* visualization, const std::vector<Frame> &frames, const Map& map);
        void draw_matches(Frame* frame1, Frame* frame2, std::vector<cv::DMatch>& matches);
        void draw_keypoint(Frame* frame);
        void draw_camera_poses(std::vector<Frame>* frames);
        void draw_ba_map(std::vector<Frame>* frames);
        void add_current_frame(std::shared_ptr<Frame> frame) {current_frame = frame;}

        std::shared_ptr<Frame> current_frame = nullptr;
    };
}

#endif //__VISUALIZATION_H_
