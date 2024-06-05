#ifndef __CAMERA_H_
#define __CAMERA_H_
#include "Eigen/Core"

namespace VISUAL_MAPPING {

    // camera model types
    enum {
        PINHOLE = 0,
        KANNALA_BRANDT8 = 1,
    }camera_model;

    class Camera {
    public:
        // pinhole camera model parameters
        class PinholeParams {
        public:
            PinholeParams() = default;
            PinholeParams(double fx, double fy, double cx, double cy, double k1, double k2, double k3, double p1, double p2):
                fx(fx), fy(fy), cx(cx), cy(cy), k1(k1), k2(k2), k3(k3), p1(p1), p2(p2) {}
            double fx, fy, cx, cy;
            double k1, k2, k3;
            double p1, p2;
        };

        // kannala brandt 8 model parameters
        class KannalaBrandt8Params{
        public:
            KannalaBrandt8Params() = default;
            KannalaBrandt8Params(double fx, double fy, double cx, double cy, double k1, double k2, double k3, double k4):
                fx(fx), fy(fy), cx(cx), cy(cy), k1(k1), k2(k2), k3(k3), k4(k4) {}
            double fx, fy, cx, cy;
            double k1, k2, k3, k4;
        };

    public:
        Camera() = default;
        Camera(int model_type, PinholeParams& pinholeParams, KannalaBrandt8Params& kannalaBrandt8Params);
        Eigen::Vector2d project(const Eigen::Vector3d &pt3d);
        Eigen::Vector3d unproject(const Eigen::Vector2d &pt2d);
        Eigen::Vector3d pixel2camera(const Eigen::Vector2d &pt2d, double depth);
        void setModelType(int modelType) { model_type = modelType; }
        [[nodiscard]] int getModelType() const { return model_type; }
        void setPinholeParams(const PinholeParams& pinholeParams) { pinhole_params = pinholeParams; }
        void setKannalaBrandt8Params(const KannalaBrandt8Params& kannalaBrandt8Params) { kannala_brandt8_params = kannalaBrandt8Params; }
        double fx() const { return pinhole_params.fx; }
        Eigen::Matrix<double,2,3> projectJac(const Eigen::Vector3d& v3D);
    private:

        Eigen::Vector2d project_pinhole(const Eigen::Vector3d &pt3d);
        Eigen::Vector3d unproject_pinhole(const Eigen::Vector2d &pt2d);

        Eigen::Vector2d project_kannala_brandt8(const Eigen::Vector3d &pt3d);
        Eigen::Vector3d unproject_kannala_brandt8(const Eigen::Vector2d &pt2d);

        int model_type = PINHOLE; // default camera model
        PinholeParams pinhole_params;
        KannalaBrandt8Params kannala_brandt8_params;
    };
}

#endif //__CAMERA_H_
