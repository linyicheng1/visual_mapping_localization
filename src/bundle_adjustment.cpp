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

    void EdgeSE3ProjectXYZOnlyStructure::linearizeOplus() {
        g2o::VertexSBAPointXYZ *vi = static_cast<g2o::VertexSBAPointXYZ *>(_vertices[0]);
        Eigen::Vector3d xyz = vi->estimate();
        Eigen::Vector3d xyz_trans = Pose_ * xyz;
        Eigen::Matrix<double, 2, 3> m = pCamera->projectJac(xyz_trans);
        _jacobianOplusXi =  - m * Pose_.rotation();
    }

    void EdgeSE3ProjectXYZ::linearizeOplus() {
        g2o::VertexSE3Expmap * vj = static_cast<g2o::VertexSE3Expmap *>(_vertices[1]);
        g2o::SE3Quat T(vj->estimate());
        g2o::VertexSBAPointXYZ* vi = static_cast<g2o::VertexSBAPointXYZ*>(_vertices[0]);
        Eigen::Vector3d xyz = vi->estimate();
        Eigen::Vector3d xyz_trans = T.map(xyz);

        double x = xyz_trans[0];
        double y = xyz_trans[1];
        double z = xyz_trans[2];

        auto projectJac = -pCamera->projectJac(xyz_trans);

        _jacobianOplusXi =  projectJac * T.rotation().toRotationMatrix();

        Eigen::Matrix<double,3,6> SE3deriv;
        SE3deriv << 0.f, z,   -y, 1.f, 0.f, 0.f,
                -z , 0.f, x, 0.f, 1.f, 0.f,
                y ,  -x , 0.f, 0.f, 0.f, 1.f;

        _jacobianOplusXj = projectJac * SE3deriv;
    }

    void BundleAdjustment::optimize_ba(const Map& map) {
        g2o::SparseOptimizer optimizer;
        optimizer.setVerbose(false);

        g2o::BlockSolver_6_3::LinearSolverType *linearSolver;
        linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

        auto *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

        auto *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        optimizer.setAlgorithm(solver);

        // add camera vertex
        int max_frame_id = 0;
        std::unordered_map<int , g2o::VertexSE3Expmap*> camera_vertex;
        for (const auto& frame : map.frames_) {
            auto * vSE3 = new g2o::VertexSE3Expmap();
            Eigen::Matrix3d Rcw = frame->get_R();
            Eigen::Vector3d tcw = frame->get_t();
            vSE3->setEstimate(g2o::SE3Quat(Rcw.transpose(), -Rcw.transpose()*tcw));
            vSE3->setId(frame->id);
            if (frame->id < 2) {
                vSE3->setFixed(true);
            } else {
                vSE3->setFixed(false);
            }
            optimizer.addVertex(vSE3);
            camera_vertex[frame->id] = vSE3;
            max_frame_id = std::max(max_frame_id, frame->id);
        }

        // add map point vertex and edge
        std::unordered_map<int, g2o::VertexSBAPointXYZ*> map_vertex;
        for (const auto& mp : map.map_points) {
            if (mp.second == nullptr) {
                continue;
            }
            auto* v = new g2o::VertexSBAPointXYZ();
            v->setId(max_frame_id + 1 + mp.second->id);
            v->setEstimate(mp.second->x3D);
            v->setMarginalized(true);
            optimizer.addVertex(v);
            map_vertex[mp.second->id] = v;

            for (int i = 0; i < mp.second->frames.size(); i++) {
                const auto &c_frame = mp.second->frames[i];
                const int c_id = mp.second->frame_feature_ids[i];
                const auto c_mp = c_frame->map_points[c_id];

                if (c_mp != mp.second) {
                    continue;
                }

                auto* e = new EdgeSE3ProjectXYZ();
                e->setVertex(0, v);
                e->setVertex(1, camera_vertex[mp.second->frames[i]->id]);
                e->setMeasurement(c_frame->features_uv[c_id]);
                e->setInformation(Eigen::Matrix2d::Identity());
                e->pCamera = mp.second->frames[i]->camera;
                optimizer.addEdge(e);
            }
        }

        // optimize
        optimizer.initializeOptimization();
        optimizer.optimize(10);

        // update frame pose
        for (const auto& frame : map.frames_) {
            g2o::VertexSE3Expmap* vSE3 = camera_vertex[frame->id];
            g2o::SE3Quat SE3quat = vSE3->estimate().inverse();
            frame->set_T(SE3quat.to_homogeneous_matrix());
        }

        // update map points
        for (const auto& mp : map.map_points) {
            if (mp.second == nullptr) {
                continue;
            }
            g2o::VertexSBAPointXYZ* v = map_vertex[mp.second->id];
            mp.second->x3D = v->estimate();
        }
    }


    void BundleAdjustment::optimize_structure(const std::vector<std::shared_ptr<Frame>>& frames,
                                              const std::vector<int>& ids,
                                              Eigen::Vector3d& xyz, std::vector<int>& status) {
//        std::cout<<"xyz: "<<xyz.transpose()<<std::endl;
        g2o::SparseOptimizer optimizer;
        optimizer.setVerbose(false);
        // 选择线性求解器
        auto* linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_3_2::PoseMatrixType>();

        // 选择块求解器
        auto* solver_ptr = new g2o::BlockSolver_3_2(linearSolver);

        // 使用Levenberg-Marquardt算法
        auto* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        optimizer.setAlgorithm(solver);

        // 添加顶点（地图点）
        auto* v = new g2o::VertexSBAPointXYZ();
        v->setId(0);
        v->setEstimate(xyz); // 初始估计值
        optimizer.addVertex(v);

        // 添加边（观测值）
        std::vector<EdgeSE3ProjectXYZOnlyStructure*> edges;
        for (int i = 0; i < frames.size(); ++i) {
            const auto& frame = frames[i];
            int id = ids[i];

            auto* e = new EdgeSE3ProjectXYZOnlyStructure();
            e->pCamera = frame->camera;
            Eigen::Isometry3d pose(frame->get_T());
            e->Pose_ = pose.inverse();
            e->setVertex(0, v);
            e->setMeasurement(frame->features_uv[id]); // 观测值

            e->setId(i);
            e->setInformation(Eigen::Matrix2d::Identity());
            optimizer.addEdge(e);
            edges.emplace_back(e);
        }
//        std::cout<<"edges size: "<<edges.size()<<std::endl;
        // 开始优化
        int num = 2;
        status.resize(frames.size(), 0);
        for (size_t it= 0;it < num; it ++)
        {
            // initial pose
            v->setEstimate(xyz); // 初始估计值

            optimizer.initializeOptimization(0);
            optimizer.optimize(10);

            status[0] = 1;
            for (int i = 1;i < frames.size();i ++) {
                const auto& e = edges[i];
                e->computeError();
                if (e->chi2() > 5.991) {
                    e->setLevel(1);
                    status[i] = 0;
                } else {
                    e->setLevel(0);
                    status[i] = 1;
                }
            }
        }
//        std::cout<<" Optimized position: "<<v->estimate().transpose()<<std::endl;
        xyz = v->estimate();
    }

    std::vector<bool> BundleAdjustment::optimize_pose(std::shared_ptr<Frame> frame) {
        g2o::SparseOptimizer optimizer;
        optimizer.setVerbose(false);
        g2o::BlockSolver_6_3::LinearSolverType *linearSolver;
        linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

        auto *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

        auto *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        optimizer.setAlgorithm(solver);

        int nInitialCorrespondences = 0;

        // 1. set frame vertex
        auto * vSE3 = new g2o::VertexSE3Expmap();
        Eigen::Matrix3d Rcw = frame->get_R();
        Eigen::Vector3d tcw = frame->get_t();
        vSE3->setEstimate(g2o::SE3Quat(Rcw.transpose(), -Rcw.transpose()*tcw));
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
            auto* e = new EdgeSE3ProjectXYZOnlyPose();

            e->setVertex(0, vSE3);
            e->setMeasurement(frame->features_uv[i]);
            e->setInformation(Eigen::Matrix2d::Identity());
            auto* rk = new g2o::RobustKernelHuber;
            rk->setDelta(2.0);
            e->setRobustKernel(rk);

            e->pCamera= frame->camera;
            e->Xw = frame->map_points[i]->x3D;
            e->setId(i);
            e->setLevel(0);

            optimizer.addEdge(e);
            vpEdgesMono.push_back(e);
        }

        if (nInitialCorrespondences < 3) {
            return inliers;
        }

        // 3. optimize
        for (size_t it= 0;it < 4; it ++)
        {
            // initial pose
            vSE3->setEstimate(g2o::SE3Quat(Rcw.transpose(), -Rcw.transpose()*tcw));
            optimizer.initializeOptimization(0);
            optimizer.optimize(10);

            int in = 0,  out = 0;
            for (auto e : vpEdgesMono) {
                e->computeError();
                if (e->chi2() > 5.991) {
                    e->setLevel(1);
                    out ++;
                } else {
                    e->setLevel(0);
                    in ++;
                }
            }
        }

        // 4. set frame pose
        g2o::SE3Quat SE3quat = vSE3->estimate().inverse();
        frame->set_T(SE3quat.to_homogeneous_matrix());
        // 5. outliers
        for (const auto& e : vpEdgesMono) {
            e->computeError();
            if (e->chi2() < 5.991 && e->isDepthPositive()) {
                inliers[e->id()] = true;
            } else {
                inliers[e->id()] = false;
            }
        }
        return inliers;
    }

} // namespace reusable_map

