/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "LoopClosing.h"
#include "Frame.h"
#include "G2oTypes.h"

#include <math.h>

#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/sparse_block_matrix.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_gauss_newton.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"

namespace ORB_SLAM3
{

class LoopClosing;

class Optimizer
{
public:

    void static BundleAdjustment(const std::vector<KeyFrame*> &vpKF, const std::vector<MapPoint*> &vpMP,
                                 int nIterations = 5, bool *pbStopFlag=NULL, const unsigned long nLoopKF=0,
                                 const bool bRobust = true);
    void static GlobalBundleAdjustemnt(Map* pMap, int nIterations=5, bool *pbStopFlag=NULL,
                                       const unsigned long nLoopKF=0, const bool bRobust = true);
    void static FullInertialBA(Map *pMap, int its, const bool bFixLocal=false, const unsigned long nLoopKF=0, bool *pbStopFlag=NULL, bool bInit=false, float priorG = 1e2, float priorA=1e6, Eigen::VectorXd *vSingVal = NULL, bool *bHess=NULL);

    void static LocalBundleAdjustment(KeyFrame* pKF, bool *pbStopFlag, Map *pMap, int& num_fixedKF, int& num_OptKF, int& num_MPs, int& num_edges);

    int static PoseOptimization(Frame* pFrame);
    int static PoseInertialOptimizationLastKeyFrame(Frame* pFrame, bool bRecInit = false);
    int static PoseInertialOptimizationLastFrame(Frame *pFrame, bool bRecInit = false);

    // if bFixScale is true, 6DoF optimization (stereo,rgbd), 7DoF otherwise (mono)
    void static OptimizeEssentialGraph(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
                                       const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                       const map<KeyFrame *, set<KeyFrame *> > &LoopConnections,
                                       const bool &bFixScale);
    void static OptimizeEssentialGraph(KeyFrame* pCurKF, vector<KeyFrame*> &vpFixedKFs, vector<KeyFrame*> &vpFixedCorrectedKFs,
                                       vector<KeyFrame*> &vpNonFixedKFs, vector<MapPoint*> &vpNonCorrectedMPs);

    // For inertial loopclosing
    void static OptimizeEssentialGraph4DoF(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
                                       const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                       const map<KeyFrame *, set<KeyFrame *> > &LoopConnections);


    // if bFixScale is true, optimize SE3 (stereo,rgbd), Sim3 otherwise (mono) (NEW)
    static int OptimizeSim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint *> &vpMatches1,
                            g2o::Sim3 &g2oS12, const float th2, const bool bFixScale,
                            Eigen::Matrix<double,7,7> &mAcumHessian, const bool bAllPoints=false);

    // For inertial systems

    void static LocalInertialBA(KeyFrame* pKF, bool *pbStopFlag, Map *pMap, int& num_fixedKF, int& num_OptKF, int& num_MPs, int& num_edges, bool bLarge = false, bool bRecInit = false);
    void static MergeInertialBA(KeyFrame* pCurrKF, KeyFrame* pMergeKF, bool *pbStopFlag, Map *pMap, LoopClosing::KeyFrameAndPose &corrPoses);

    // Local BA in welding area when two maps are merged
    void static LocalBundleAdjustment(KeyFrame* pMainKF,vector<KeyFrame*> vpAdjustKF, vector<KeyFrame*> vpFixedKF, bool *pbStopFlag);

    // Marginalize block element (start:end,start:end). Perform Schur complement.
    // Marginalized elements are filled with zeros.
    static Eigen::MatrixXd Marginalize(const Eigen::MatrixXd &H, const int &start, const int &end);

    // Inertial pose-graph
    void static InertialOptimization(Map *pMap, Eigen::Matrix3d &Rwg, double &scale, Eigen::Vector3d &bg, Eigen::Vector3d &ba, bool bMono, Eigen::MatrixXd  &covInertial, bool bFixedVel=false, bool bGauss=false, float priorG = 1e2, float priorA = 1e6);
    void static InertialOptimization(Map *pMap, Eigen::Vector3d &bg, Eigen::Vector3d &ba, float priorG = 1e2, float priorA = 1e6);
    void static InertialOptimization(Map *pMap, Eigen::Matrix3d &Rwg, double &scale);

    // acoustic related optimization
    void static PoseOptimizationDistanceGivenScale(Eigen::Vector3d &pose_est, double scale, vector<Eigen::Vector3d> pose_others, vector<double> distances); 
    void static PoseOptimizationDistanceRegu(Eigen::Vector3d &pose_est, Eigen::Vector3d pose_last, double scale, vector<Eigen::Vector3d> pose_others, vector<double> distances); 
    void static IMUAcousticOptimization(vector<Eigen::Vector3d> &pos_est, vector<Eigen::Vector3d> &vel_est, vector<Eigen::Vector3d> delta_pos, vector<Eigen::Vector3d> delta_vel, double scale, vector<Eigen::Vector3d> pose_others, vector<double> distances);
    void static IMUAcousticKeyOptimization(vector<Eigen::Vector3d> &pos_est, vector<Eigen::Vector3d> delta_p_est, vector<vector<double>> distances, double scale, vector<Eigen::Vector3d> pose_others); 

    // calibration for microphone
    void static CalibOptimization(Eigen::Vector3d &t_mc, double* scale, vector<Sophus::SE3d> pose_u0, vector<Sophus::SE3d> pose_others, vector<vector<double>> distances);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};


// extension for optimize pose with acoustic ranging results

class VertexTran : public g2o::BaseVertex<3, Eigen::Vector3d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        virtual void setToOriginImpl() override {
        _estimate << 0, 0, 0;
    }

    virtual void oplusImpl(const double* update) override {
        _estimate += Eigen::Vector3d(update);
    }

    virtual bool read(istream& in) {}

    virtual bool write(ostream& out) const {}
};

class VertexScaleSimple : public g2o::BaseVertex<1, double> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        virtual void setToOriginImpl() override {
        _estimate = 1;
    }

    virtual void oplusImpl(const double* update) override {
        _estimate += update[0];
    }

    virtual bool read(istream& in) {}

    virtual bool write(ostream& out) const {}
};

// unit edge with pose, considering distance
class EdgeDistS : public g2o::BaseUnaryEdge<1, double, VertexTran> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeDistS(Eigen::Vector3d pos, double scale): BaseUnaryEdge(), _pos(pos), _scale(scale)
    {}

    virtual void computeError() override {
        const VertexTran* v = static_cast<VertexTran*>(_vertices[0]);
        Eigen::Vector3d T = v->estimate();
        //cout << "T " << T << endl;
        double s = _scale; // scale converts from slam to real world
        _error(0, 0) = _measurement - s*sqrt((T-_pos).dot(T-_pos));
        
        //cout << "measure " << _measurement << endl;
        //cout << "calculate error " << _error << endl; 
    }
    
    /*virtual void linearizeOplus() override {
        const VertexTran* v = static_cast<VertexTran*>(_vertices[0]);
        Vector3d T = v->estimate();
        double s = _scale(); 
        _jacobianOplusXi[0] = (-2 * s * s * (T(0, 0)-_pos(0,0)));
        _jacobianOplusXi[1] = (-2 * s * s * (T(1, 0)-_pos(1,0)));
        _jacobianOplusXi[2] = (-2 * s * s * (T(2, 0)-_pos(2,0)));
        _jacobianOplusXj[0] = -2 * s * (T-_pos).dot(T-_pos);
    }*/
    
    virtual bool read(std::istream& in) override { return true; }

    virtual bool write(std::ostream& out) const override { return true; }

private:
  Eigen::Vector3d _pos; // position of the other user
  double _scale; // scale
};

class Edge3d : public g2o::BaseBinaryEdge<3,Eigen::Vector3d, VertexTran, VertexTran>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Edge3d(): BaseBinaryEdge()
    {}

    virtual void computeError() override {
        // time slot t
        const VertexTran* v1 = static_cast<VertexTran*>(_vertices[0]);
        Eigen::Vector3d T1 = v1->estimate();
        // time slot t-1
        const VertexTran* v2 = static_cast<VertexTran*>(_vertices[1]);
        Eigen::Vector3d T2 = v2->estimate();
        _error << _measurement - (T1-T2);
        
        //cout << "measure " << _measurement << endl;
        //cout << "calculate error " << _error << endl; 
    }

    virtual bool read(std::istream& in) override { return true; }

    virtual bool write(std::ostream& out) const override { return true; }
private:
};

class EdgeCalib : public g2o::BaseBinaryEdge<1,double, VertexTran, VertexScale>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeCalib(Sophus::SE3d pose0, Sophus::SE3d pose1): BaseBinaryEdge(), pos0_(pose0), pos1_(pose1)
    {}

    virtual void computeError() override {
        // scale converts from real world to slam
        const VertexTran* v1 = static_cast<VertexTran*>(_vertices[0]);
        Eigen::Vector3d t_mc = v1->estimate();
        const VertexScale* v2 = static_cast<VertexScale*>(_vertices[1]);
        double s = v2->estimate();
        Eigen::Vector3d t_wm_0=pos0_.rotationMatrix()*(-s*t_mc)+pos0_.translation();
        Eigen::Vector3d t_wm_1=pos1_.rotationMatrix()*(-s*t_mc)+pos1_.translation();

        _error(0, 0) = _measurement - (t_wm_0-t_wm_1).norm()/s; // error needs to be in the scale of real world
        //cout << "measure " << _measurement << endl;
        //cout << "calculate error " << _error << endl; 
    }

    virtual bool read(std::istream& in) override { return true; }

    virtual bool write(std::ostream& out) const override { return true; }
private:
    Sophus::SE3d pos0_;
    Sophus::SE3d pos1_;
};



} //namespace ORB_SLAM3

#endif // OPTIMIZER_H
