//
// Created by peyschen on 10/02/23.
//

#include "open3d_slam/Plane.hpp"
#include <Eigen/Eigenvalues>
namespace o3d_slam {
    void Plane::initialize(const std::vector<PointWithCov> &pts) {
        planeCov_ = Matrix6d::Zero();
        cov_ = Eigen::Matrix3d::Zero();
        center_ = Eigen::Vector3d::Zero();
        normal_ = Eigen::Vector3d::Zero();

        numPoints_ = pts.size();
        for (auto p:pts) {
            cov_ += p.point * p.point.transpose();
            center_ += p.point;
        }
        center_ /= pts.size();
        cov_ = cov_ / pts.size()-center_ * center_.transpose();
        Eigen::EigenSolver<Eigen::Matrix3d> solver(cov_);
        Eigen::Matrix3d realEigVecs = solver.eigenvectors().real();
        Eigen::Vector3d realEigVals = solver.eigenvalues().real();

        Eigen::Index minEigVal,maxEigVal,midEigVal;
        realEigVals.minCoeff(&minEigVal);
        realEigVals.maxCoeff(&maxEigVal);
        midEigVal = 3 - minEigVal - maxEigVal;

        Eigen::Matrix3d J_Q = Eigen::Matrix3d::Identity();
        J_Q /= pts.size();
        if(realEigVals(minEigVal) < planeThreshold_){
            std::vector<int> index(pts.size());
            for (int i = 0; i < pts.size(); i++) {
                Eigen::Matrix<double, 6, 3> J;
                Eigen::Matrix3d F;
                for (int m = 0; m < 3; m++) {
                    if (m != (int)minEigVal) {
                        Eigen::Matrix<double, 1, 3> F_m =
                                (pts[i].point - center_).transpose() /
                                (pts.size() * (realEigVals[minEigVal] - realEigVals[m])) *
                                (realEigVecs.col(m) * realEigVecs.col(minEigVal).transpose() +
                                        realEigVecs.col(minEigVal) * realEigVecs.col(m).transpose());
                        F.row(m) = F_m;
                    } else {
                        Eigen::Matrix<double, 1, 3> F_m;
                        F_m << 0, 0, 0;
                        F.row(m) = F_m;
                    }
                }
                J.block<3, 3>(0, 0) = realEigVecs * F;
                J.block<3, 3>(3, 0) = J_Q;
                planeCov_ += J * pts[i].cov * J.transpose();
            }
            isPlane = true;
        } else{
            isPlane = false;
        }
        normal_ << realEigVecs(0, minEigVal), realEigVecs(1, minEigVal),
                realEigVecs(2, minEigVal);
        v_<< realEigVecs(0, midEigVal), realEigVecs(1, midEigVal),
                realEigVecs(2, midEigVal);
        u_ << realEigVecs(0, maxEigVal), realEigVecs(1, maxEigVal),
                realEigVecs(2, maxEigVal);
        eigenValues_[0] = realEigVals(minEigVal);
        eigenValues_[1] = realEigVals(midEigVal);
        eigenValues_[2] = realEigVals(maxEigVal);

        if (!isInitialized) {
            isInitialized = true;
        }


    }
    void Plane::update(const std::vector<PointWithCov>&pts){
        Eigen::Matrix3d oldCov = cov_;
        Eigen::Vector3d oldCenter = center_;
        Eigen::Matrix3d sumPpt = (cov_ + center_ * center_.transpose()) * numPoints_;
        Eigen::Vector3d sumP = center_ * numPoints_;
        for(const auto& pt:pts){
            sumPpt += pt.point * pt.point.transpose();
            sumP += pt.point;
        }
        numPoints_ += pts.size();
        center_ = sumP/numPoints_;
        cov_ = sumPpt / numPoints_ - center_ * center_.transpose();

        Eigen::EigenSolver<Eigen::Matrix3d> solver(cov_);
        Eigen::Matrix3d realEigVecs = solver.eigenvectors().real();
        Eigen::Vector3d realEigVals = solver.eigenvalues().real();

        Eigen::Index minEigVal,maxEigVal,midEigVal;
        realEigVals.minCoeff(&minEigVal);
        realEigVals.maxCoeff(&maxEigVal);
        midEigVal = 3 - minEigVal - maxEigVal;

        normal_ << realEigVecs(0, minEigVal), realEigVecs(1, minEigVal),
                realEigVecs(2, minEigVal);
        v_<< realEigVecs(0, midEigVal), realEigVecs(1, midEigVal),
                realEigVecs(2, midEigVal);
        u_ << realEigVecs(0, maxEigVal), realEigVecs(1, maxEigVal),
                realEigVecs(2, maxEigVal);
        eigenValues_[0] = realEigVals(minEigVal);
        eigenValues_[1] = realEigVals(midEigVal);
        eigenValues_[2] = realEigVals(maxEigVal);
        isPlane = (realEigVals(minEigVal) < planeThreshold_);
    }
}
