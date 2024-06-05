#include "bundle_adjustment.h"

namespace VISUAL_MAPPING {

    bool EdgeSE3ProjectXYZOnlyPose::read(std::istream& is){
        for (int i=0; i<2; i++){
            is >> _measurement[i];
        }
        for (int i=0; i<2; i++)
            for (int j=i; j<2; j++) {
                is >> information()(i,j);
                if (i!=j)
                    information()(j,i)=information()(i,j);
            }
        return true;
    }

    bool EdgeSE3ProjectXYZOnlyPose::write(std::ostream& os) const {

        for (int i=0; i<2; i++){
            os << measurement()[i] << " ";
        }

        for (int i=0; i<2; i++)
            for (int j=i; j<2; j++){
                os << " " <<  information()(i,j);
            }
        return os.good();
    }


    void EdgeSE3ProjectXYZOnlyPose::linearizeOplus() {
        g2o::VertexSE3Expmap * vi = static_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
        Eigen::Vector3d xyz_trans = vi->estimate().map(Xw);

        double x = xyz_trans[0];
        double y = xyz_trans[1];
        double z = xyz_trans[2];

        Eigen::Matrix<double,3,6> SE3deriv;
        SE3deriv << 0.f, z,   -y, 1.f, 0.f, 0.f,
                -z , 0.f, x, 0.f, 1.f, 0.f,
                y ,  -x , 0.f, 0.f, 0.f, 1.f;

        _jacobianOplusXi = -pCamera->projectJac(xyz_trans) * SE3deriv;
    }

    void BundleAdjustment::optimize_ba(std::vector<Frame> *frames) {

    }

    void BundleAdjustment::optimize_structure(std::vector<Frame> *frames) {

    }

    std::vector<bool> BundleAdjustment::optimize_pose(Frame& frame) {
        g2o::SparseOptimizer optimizer;
        g2o::BlockSolver_6_3::LinearSolverType *linearSolver;
        linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

        g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

        g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        optimizer.setAlgorithm(solver);

        int nInitialCorrespondences = 0;

        // 1. set frame vertex
        auto * vSE3 = new g2o::VertexSE3Expmap();
        Eigen::Matrix3d Rcw = frame.get_R();
        Eigen::Vector3d tcw = frame.get_t();
        vSE3->setEstimate(g2o::SE3Quat(Rcw, tcw));
        vSE3->setId(0);
        vSE3->setFixed(false);
        optimizer.addVertex(vSE3);

        // 2. set map point vertex
        std::vector<EdgeSE3ProjectXYZOnlyPose*> vpEdgesMono;
        std::vector<bool> inliers;
        vpEdgesMono.reserve(frame.map_points.size());
        inliers.resize(frame.map_points.size(), false);
        for (int i = 0; i < frame.map_points.size(); i++) {
            if (frame.map_points[i] == nullptr) {
                continue;
            }
            nInitialCorrespondences++;
            EdgeSE3ProjectXYZOnlyPose* e = new EdgeSE3ProjectXYZOnlyPose();

            e->setVertex(0, vSE3);
            e->setMeasurement(frame.features_uv[i]);
            e->setInformation(Eigen::Matrix2d::Identity());
            auto* rk = new g2o::RobustKernelHuber;
            rk->setDelta(1.0);
            e->setRobustKernel(rk);

            e->pCamera= frame.camera;
            e->Xw = frame.map_points[i]->x3D;
            e->setId(i);

            optimizer.addEdge(e);
            vpEdgesMono.push_back(e);
        }

        // 3. optimize
        for (size_t it= 0;it < 4; it ++)
        {
            // initial pose
            vSE3->setEstimate(g2o::SE3Quat(Rcw, tcw));

            optimizer.initializeOptimization(0);
            optimizer.optimize(10);

            for (auto e : vpEdgesMono) {
                e->computeError();
                if (e->chi2() > 5.991 || !e->isDepthPositive()) {
                    e->setLevel(1);
                } else {
                    e->setLevel(0);
                }
            }
        }

        // 4. set frame pose
        g2o::SE3Quat SE3quat = vSE3->estimate();
        frame.set_T(SE3quat.to_homogeneous_matrix());
        // 5. outliers
        for (const auto& e : vpEdgesMono) {
            e->computeError();
            if (e->chi2() < 5.991 && e->isDepthPositive()) {
                inliers[e->id()] = true;
            }
        }
        return inliers;
    }

} // namespace reusable_map

