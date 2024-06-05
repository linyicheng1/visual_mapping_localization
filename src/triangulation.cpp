#include <iostream>
#include "triangulation.h"

namespace VISUAL_MAPPING {
    void Triangulation::triangulate(Frame *frame1, Frame *frame2, std::vector<cv::DMatch> &matches)
    {

    }

    std::pair<double, double> Triangulation::triangulate(Camera *camera1, Camera *camera2,
                                      const Eigen::Vector2d &kp1, const Eigen::Vector2d &kp2,
                                      const Eigen::Matrix3d &R, const Eigen::Vector3d &t)
    {
        Eigen::Vector3d r1 = camera1->unproject(kp1);
        Eigen::Vector3d r2 = camera2->unproject(kp2);
//        std::cout<<"r1 "<<r1<<std::endl;
//        std::cout<<"r2 "<<r2<<std::endl;

        // 1. Check parallax
        Eigen::Vector3d r21= R * r2;
        const double cosParallaxRays = r1.dot(r21) / (r1.norm() * r21.norm());
        if (cosParallaxRays > 0.9998) {
            return std::make_pair(-1, 100);
        }

        // 2. try to triangulate
        Eigen::Vector3d x3D;
        Eigen::Matrix3d R21 = R.transpose();
        Eigen::Vector3d t21 = -R21 * t;

        Eigen::Matrix4d A;
        Eigen::Matrix<double,3,4> Tcw1;
        Eigen::Matrix<double,3,4> Tcw2;
        Tcw1 << Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero();
        Tcw2 << R21, t21;
        A.row(0) = r1(0) * Tcw1.row(2) - Tcw1.row(0);
        A.row(1) = r1(1) * Tcw1.row(2) - Tcw1.row(1);
        A.row(2) = r2(0) * Tcw2.row(2) - Tcw2.row(0);
        A.row(3) = r2(1) * Tcw2.row(2) - Tcw2.row(1);
//        std::cout<<" A "<<A<<std::endl;
        Eigen::JacobiSVD<Eigen::Matrix4d> svd(A, Eigen::ComputeFullV);
        Eigen::Vector4d x3D_homogeneous = svd.matrixV().col(3);
        x3D = x3D_homogeneous.head(3) / x3D_homogeneous(3);
//        std::cout<<"x3D "<<x3D<<std::endl;
        // 3.1 check depth
        double z1 = x3D.z();
        if (z1 < 0.0) {
            return std::make_pair(-2, 100);
        }
        double z2 = R21.row(2).dot(x3D) + t21.z();
        if (z2 < 0.0) {
            return std::make_pair(-3, 100);
        }
        // 3.2 check reprojection error
//        Eigen::Vector2d uv1 = camera1->project(x3D);
//        double error1 = (uv1 - kp1).norm();
//
//        Eigen::Vector3d x3D2 = R21 * x3D + t21;
//        Eigen::Vector2d uv2 = camera2->project(x3D2);
//        double error2 = (uv2 - kp2).norm();
//
//        if (error1 > 5.991 || error2 > 5.991) {
//            return -4;
//        }

        // uncertainty
        double uncertainty = 0.0;
        Eigen::Vector3d a= r1 * z1 - t;
        double t_norm = t.norm();
        double a_norm = a.norm();
        double alpha = acos(r1.dot(t) / (t_norm * r1.norm()));
        double beta = acos(a.dot(-t) / (a_norm * t_norm));

        double uv_norm = kp2.norm();
        double px_error_angle = atan((uv_norm + 1) / (camera2->fx())) - atan(uv_norm / (camera2->fx()));
        double beta_plus = beta + px_error_angle;
        double gamma_plus = M_PI-alpha-beta_plus;
        double z_plus = t_norm * sin(beta_plus) / sin(gamma_plus);
        uncertainty = (z_plus - z1);
        return std::make_pair(z1, uncertainty);
    }

}
