#ifndef __BUNDLE_ADJUSTMENT_H_
#define __BUNDLE_ADJUSTMENT_H_
#include "frame.h"
#include "map_point.h"
#include "Eigen/Dense"
#include "Thirdparty/g2o/g2o/core/sparse_block_matrix.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_gauss_newton.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"


namespace VISUAL_MAPPING {


    class EdgeSE3ProjectXYZOnlyStructure: public g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ>{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        EdgeSE3ProjectXYZOnlyStructure(){}

        virtual void computeError() override {
            const g2o::VertexSBAPointXYZ* v0 = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);
            Eigen::Vector2d obs(_measurement);
            _error = obs - pCamera->project(Pose_ * v0->estimate());
        }

        virtual void linearizeOplus() override;

        bool isDepthPositive() {
            const g2o::VertexSBAPointXYZ* v0 = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);
            return (Pose_ * v0->estimate())(2)>0.0;
        }

        virtual bool read(std::istream& in) override {return true;}
        virtual bool write(std::ostream& out) const override {return true;}
        Camera* pCamera;
        Eigen::Isometry3d Pose_;
    };

    class  EdgeSE3ProjectXYZOnlyPose: public  g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE3Expmap>{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EdgeSE3ProjectXYZOnlyPose(){}

        bool read(std::istream& is);

        bool write(std::ostream& os) const;

        void computeError()  {
            const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
            Eigen::Vector2d obs(_measurement);
            _error = obs-pCamera->project(v1->estimate().map(Xw));
        }

        bool isDepthPositive() {
            const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
            return (v1->estimate().map(Xw))(2)>0.0;
        }


        virtual void linearizeOplus();

        Eigen::Vector3d Xw;
        Camera* pCamera;
    };

    class EdgeSE3ProjectXYZ: public g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ, g2o::VertexSE3Expmap> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EdgeSE3ProjectXYZ(){}
        virtual bool read(std::istream& in) override {return true;}
        virtual bool write(std::ostream& out) const override {return true;}

        void computeError()  {
            const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
            const g2o::VertexSBAPointXYZ* v2 = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);
            Eigen::Vector2d obs(_measurement);
            _error = obs-pCamera->project(v1->estimate().map(v2->estimate()));
        }

        bool isDepthPositive() {
            const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
            const g2o::VertexSBAPointXYZ* v2 = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);
            return ((v1->estimate().map(v2->estimate()))(2)>0.0);
        }

        virtual void linearizeOplus();

        Camera* pCamera;
    };

    class BundleAdjustment {
    public:
        BundleAdjustment() = default;
        std::vector<bool> optimize_pose(std::shared_ptr<Frame> frame);
        void optimize_ba(const Map& frames);
        void optimize_structure(const std::vector<std::shared_ptr<Frame>>& frames, const std::vector<int>& ids,
                                Eigen::Vector3d& xyz, std::vector<int>& status);
    private:

    };

}

#endif //__BUNDLE_ADJUSTMENT_H_
