//
// Created by peyschen on 10/02/23.
//

#ifndef OPEN3D_SLAM_PLANE_H
#define OPEN3D_SLAM_PLANE_H

#include <Eigen/Core>
#include "realtime_meshing/types.h"

class Plane {
public:
    void initialize(const std::vector<Eigen::Vector3d> &pts);

    void update(const std::vector<Eigen::Vector3d> &pts);

    bool isPlane = false;
    bool isInitialized = false;
    int id;

    Eigen::Vector3d getPlaneCenter() { return center_; };

    Eigen::Vector3d getPlaneNormal() { return normal_; };

    Eigen::Matrix<double, 3, 2> getTangentialBase() {
        Eigen::Matrix<double, 3, 2> base;
        base << u_, v_;
        return base;
    };

private:
    Eigen::Matrix3d cov_ = Eigen::Matrix3d::Zero();
    Eigen::Vector3d center_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d normal_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d u_, v_;
    double eigenValues_[3];
    float planeThreshold_;
    int numPoints_;
};

#endif  // OPEN3D_SLAM_PLANE_H
