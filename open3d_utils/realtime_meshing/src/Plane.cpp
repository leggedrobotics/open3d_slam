//
// Created by peyschen on 10/02/23.
//

#include "realtime_meshing/Plane.hpp"
#include <Eigen/Eigenvalues>

void Plane::initialize(const std::vector<Eigen::Vector3d> &pts) {
    cov_ = Eigen::Matrix3d::Zero();
    center_ = Eigen::Vector3d::Zero();
    normal_ = Eigen::Vector3d::Zero();

    numPoints_ = pts.size();
    for (auto p: pts) {
        cov_ += p * p.transpose();
        center_ += p;
    }
    center_ /= pts.size();
    cov_ = cov_ / pts.size() - center_ * center_.transpose();
    Eigen::EigenSolver<Eigen::Matrix3d> solver(cov_);
    Eigen::Matrix3d realEigVecs = solver.eigenvectors().real();
    Eigen::Vector3d realEigVals = solver.eigenvalues().real();

    Eigen::Index minEigVal;
    Eigen::Index maxEigVal;
    Eigen::Index midEigVal;
    realEigVals.minCoeff(&minEigVal);
    realEigVals.maxCoeff(&maxEigVal);
    midEigVal = 3 - minEigVal - maxEigVal;

    if (realEigVals(minEigVal) < planeThreshold_) {
        isPlane = true;
    } else {
        isPlane = false;
    }
    normal_ << realEigVecs(0, minEigVal), realEigVecs(1, minEigVal),
            realEigVecs(2, minEigVal);
    v_ << realEigVecs(0, midEigVal), realEigVecs(1, midEigVal),
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

void Plane::update(const std::vector<Eigen::Vector3d> &pts) {
    Eigen::Matrix3d sumPpt = (cov_ + center_ * center_.transpose()) * numPoints_;
    Eigen::Vector3d sumP = center_ * numPoints_;
    for (const auto &pt: pts) {
        sumPpt += pt * pt.transpose();
        sumP += pt;
    }
    numPoints_ += pts.size();
    center_ = sumP / numPoints_;
    cov_ = sumPpt / numPoints_ - center_ * center_.transpose();

    Eigen::EigenSolver<Eigen::Matrix3d> solver(cov_);
    Eigen::Matrix3d realEigVecs = solver.eigenvectors().real();
    Eigen::Vector3d realEigVals = solver.eigenvalues().real();

    Eigen::Index minEigVal;
    Eigen::Index maxEigVal;
    Eigen::Index midEigVal;
    realEigVals.minCoeff(&minEigVal);
    realEigVals.maxCoeff(&maxEigVal);
    midEigVal = 3 - minEigVal - maxEigVal;

    normal_ << realEigVecs(0, minEigVal), realEigVecs(1, minEigVal),
            realEigVecs(2, minEigVal);
    v_ << realEigVecs(0, midEigVal), realEigVecs(1, midEigVal),
            realEigVecs(2, midEigVal);
    u_ << realEigVecs(0, maxEigVal), realEigVecs(1, maxEigVal),
            realEigVecs(2, maxEigVal);
    eigenValues_[0] = realEigVals(minEigVal);
    eigenValues_[1] = realEigVals(midEigVal);
    eigenValues_[2] = realEigVals(maxEigVal);
    isPlane = (realEigVals(minEigVal) < planeThreshold_);
}
