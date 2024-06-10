#include "bundle_adjustment.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"

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


    void BundleAdjustment::optimize_structure(const std::vector<std::shared_ptr<Frame>>& frames,
                                              const std::vector<int>& ids,
                                              Eigen::Vector3d& xyz, std::vector<int>& status) {
        g2o::SparseOptimizer optimizer;
        // 选择线性求解器
        g2o::BlockSolverX::LinearSolverType* linearSolver =
                new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

        // 选择块求解器
        auto* solver_ptr = new g2o::BlockSolverX(linearSolver);

        // 使用Levenberg-Marquardt算法
        auto* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        optimizer.setAlgorithm(solver);

        // 添加顶点（地图点）
        auto* v = new VertexPointXYZ();
        v->setId(0);
        v->setEstimate(xyz); // 初始估计值
        optimizer.addVertex(v);

        // 添加边（观测值）
        for (int i = 0; i < frames.size(); ++i) {
            const auto& frame = frames[i];
            int id = ids[i];
            if (frame->map_points[id] == nullptr) {
                continue;
            }
            Eigen::Isometry3d pose(frame->get_T());

            auto* e = new EdgeSE3ProjectXYZOnlyStructure();
            e->pose_ = pose;
            e->pCamera = frame->camera;

            e->setVertex(0, v);
            e->setMeasurement(frame->features_uv[id]); // 观测值

            e->setInformation(Eigen::Matrix2d::Identity());
            optimizer.addEdge(e);
        }

        // 开始优化
        optimizer.initializeOptimization();
        optimizer.optimize(10);
    }

    std::vector<bool> BundleAdjustment::optimize_pose(std::shared_ptr<Frame> frame) {
        g2o::SparseOptimizer optimizer;
        g2o::BlockSolver_6_3::LinearSolverType *linearSolver;
        linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

        auto *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

        g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        optimizer.setAlgorithm(solver);

        int nInitialCorrespondences = 0;

        // 1. set frame vertex
        auto * vSE3 = new g2o::VertexSE3Expmap();
        Eigen::Matrix3d Rcw = frame->get_R();
        Eigen::Vector3d tcw = frame->get_t();
        vSE3->setEstimate(g2o::SE3Quat(Rcw, tcw));
        vSE3->setId(0);
        vSE3->setFixed(false);
        optimizer.addVertex(vSE3);

        // 2. set map point vertex
        std::vector<EdgeSE3ProjectXYZOnlyPose*> vpEdgesMono;
        std::vector<bool> inliers;
        vpEdgesMono.reserve(frame->map_points.size());
        inliers.resize(frame->map_points.size(), false);
        for (int i = 0; i < frame->map_points.size(); i++) {
            if (frame->map_points[i] == nullptr) {
                continue;
            }
            nInitialCorrespondences++;
            EdgeSE3ProjectXYZOnlyPose* e = new EdgeSE3ProjectXYZOnlyPose();

            e->setVertex(0, vSE3);
            e->setMeasurement(frame->features_uv[i]);
            e->setInformation(Eigen::Matrix2d::Identity());
            auto* rk = new g2o::RobustKernelHuber;
            rk->setDelta(1.0);
            e->setRobustKernel(rk);

            e->pCamera= frame->camera;
            e->Xw = frame->map_points[i]->x3D;
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
        frame->set_T(SE3quat.to_homogeneous_matrix());
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

