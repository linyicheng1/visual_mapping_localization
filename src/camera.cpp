#include "camera.h"

namespace VISUAL_MAPPING {

    Camera::Camera(int model_type, Camera::PinholeParams &pinholeParams,
                   Camera::KannalaBrandt8Params &kannalaBrandt8Params) {
        model_type = model_type;
        pinhole_params = pinholeParams;
        kannala_brandt8_params = kannalaBrandt8Params;
    }

    Eigen::Vector2d Camera::project(const Eigen::Vector3d &pt3d) {
        switch (model_type) {
            case PINHOLE:
                return project_pinhole(pt3d);
            case KANNALA_BRANDT8:
                return project_kannala_brandt8(pt3d);
        }
    }

    Eigen::Vector3d Camera::unproject(const Eigen::Vector2d &pt2d) {
        switch (model_type) {
            case PINHOLE:
                return unproject_pinhole(pt2d);
            case KANNALA_BRANDT8:
                return unproject_kannala_brandt8(pt2d);
        }
    }

    Eigen::Vector2d Camera::project_pinhole(const Eigen::Vector3d &pt3d) {
        Eigen::Vector2d pt2d;
        pt2d[0] = pinhole_params.fx * pt3d[0] / pt3d[2] + pinhole_params.cx;
        pt2d[1] = pinhole_params.fy * pt3d[1] / pt3d[2] + pinhole_params.cy;
        return pt2d;
    }

    Eigen::Vector3d Camera::unproject_pinhole(const Eigen::Vector2d &pt2d) {
        Eigen::Vector3d pt3d;
        pt3d[0] = (pt2d[0] - pinhole_params.cx) / pinhole_params.fx;
        pt3d[1] = (pt2d[1] - pinhole_params.cy) / pinhole_params.fy;
        pt3d[2] = 1.0;
        return pt3d;
    }

    Eigen::Vector2d Camera::project_kannala_brandt8(const Eigen::Vector3d &pt3d) {
        const double x2_plus_y2 = pt3d.x() * pt3d.x() + pt3d.y() * pt3d.y();
        const double theta = atan2(sqrt(x2_plus_y2), pt3d.z());
        const double psi = atan2(pt3d.y(), pt3d.x());

        const double theta2 = theta * theta;
        const double theta3 = theta * theta2;
        const double theta5 = theta3 * theta2;
        const double theta7 = theta5 * theta2;
        const double theta9 = theta7 * theta2;
        const double r = theta + kannala_brandt8_params.k1 * theta3 + kannala_brandt8_params.k2 * theta5
                        + kannala_brandt8_params.k3 * theta7 + kannala_brandt8_params.k4 * theta9;

        Eigen::Vector2d uv(kannala_brandt8_params.fx * r * cos(psi) + kannala_brandt8_params.cx,
                           kannala_brandt8_params.fy * r * sin(psi) + kannala_brandt8_params.cy);
        return uv;
    }

    Eigen::Vector3d Camera::unproject_kannala_brandt8(const Eigen::Vector2d &pt2d) {

        Eigen::Vector2d pw((pt2d.x() - kannala_brandt8_params.cx) / kannala_brandt8_params.fx,
                           (pt2d.y() - kannala_brandt8_params.cy) / kannala_brandt8_params.fy);

        double scale = 1.;
        double theta_d = sqrt(pw[0] * pw[0] + pw[1] * pw[1]);
        theta_d = std::min(std::max(-M_PI/2, theta_d), M_PI/2);

        if (theta_d > 1e-8) {
            double theta = theta_d;
            for (int j = 0; j < 10; j ++) {
                double theta2 = theta * theta, theta4 = theta2 * theta2, theta6 = theta4 * theta2, theta8 =
                        theta4 * theta4;
                double k0_theta2 = kannala_brandt8_params.k1 * theta2, k1_theta4 = kannala_brandt8_params.k2 * theta4;
                double k2_theta6 = kannala_brandt8_params.k3 * theta6, k3_theta8 = kannala_brandt8_params.k4 * theta8;
                double theta_fix = (theta * (1 + k0_theta2 + k1_theta4 + k2_theta6 + k3_theta8) - theta_d) /
                                  (1 + 3 * k0_theta2 + 5 * k1_theta4 + 7 * k2_theta6 + 9 * k3_theta8);
                theta = theta - theta_fix;
                if (std::abs(theta_fix) < 1e-6)
                    break;
            }
            scale = std::tan(theta) / theta_d;
        }
        Eigen::Vector3d p3d(pw[0] * scale, pw[1] * scale, 1.);
        return p3d;
    }

    Eigen::Vector3d Camera::pixel2camera(const Eigen::Vector2d &pt2d, double depth) {
        return unproject(pt2d) * depth;
    }

    Eigen::Matrix<double, 2, 3> Camera::projectJac(const Eigen::Vector3d &v3D) {
        double x2 = v3D[0] * v3D[0], y2 = v3D[1] * v3D[1], z2 = v3D[2] * v3D[2];
        double r2 = x2 + y2;
        double r = sqrt(r2);
        double r3 = r2 * r;
        double theta = atan2(r, v3D[2]);

        double theta2 = theta * theta, theta3 = theta2 * theta;
        double theta4 = theta2 * theta2, theta5 = theta4 * theta;
        double theta6 = theta2 * theta4, theta7 = theta6 * theta;
        double theta8 = theta4 * theta4, theta9 = theta8 * theta;

        double f = theta + theta3 * kannala_brandt8_params.k1 + theta5 * kannala_brandt8_params.k1 + theta7 * kannala_brandt8_params.k3 +
                   theta9 * kannala_brandt8_params.k4;
        double fd = 1 + 3 * kannala_brandt8_params.k1 * theta2 + 5 * kannala_brandt8_params.k2 * theta4 + 7 * kannala_brandt8_params.k3 * theta6 +
                    9 * kannala_brandt8_params.k4 * theta8;

        Eigen::Matrix<double, 2, 3> JacGood;
        JacGood(0, 0) = kannala_brandt8_params.fx * (fd * v3D[2] * x2 / (r2 * (r2 + z2)) + f * y2 / r3);
        JacGood(1, 0) =
                kannala_brandt8_params.fy * (fd * v3D[2] * v3D[1] * v3D[0] / (r2 * (r2 + z2)) - f * v3D[1] * v3D[0] / r3);

        JacGood(0, 1) =
                kannala_brandt8_params.fx * (fd * v3D[2] * v3D[1] * v3D[0] / (r2 * (r2 + z2)) - f * v3D[1] * v3D[0] / r3);
        JacGood(1, 1) = kannala_brandt8_params.fy * (fd * v3D[2] * y2 / (r2 * (r2 + z2)) + f * x2 / r3);

        JacGood(0, 2) = -kannala_brandt8_params.fx * fd * v3D[0] / (r2 + z2);
        JacGood(1, 2) = -kannala_brandt8_params.fy * fd * v3D[1] / (r2 + z2);

        return JacGood;
    }

} // namespace reusable_map
