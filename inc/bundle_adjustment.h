#ifndef __BUNDLE_ADJUSTMENT_H_
#define __BUNDLE_ADJUSTMENT_H_
#include "frame.h"
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

    class BundleAdjustment {
    public:
        BundleAdjustment() = default;
        std::vector<bool> optimize_pose(Frame& frame);
        void optimize_ba(std::vector<Frame>* frames);
        void optimize_structure(std::vector<Frame>* frames);
    private:

    };

}

#endif //__BUNDLE_ADJUSTMENT_H_
