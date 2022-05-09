
#pragma once

#include <Eigen/Core>
#include <tuple>
#include <vector>

#include "open3d/pipelines/registration/Registration.h"
#include "open3d_slam/typedefs.hpp"
#include "open3d_slam/TransformationEstimation.h"
#include "open3d_slam/Voxel.hpp"

namespace o3d_slam {

using ICPConvergenceCriteria = open3d::pipelines::registration::ICPConvergenceCriteria;
using CorrespondenceSet = open3d::pipelines::registration::CorrespondenceSet;
using RegistrationResult = open3d::pipelines::registration::RegistrationResult;


class RegistrationResultVoxelized {
public:
    /// \brief Parameterized Constructor.
    ///
    /// \param transformation The estimated transformation matrix.
	RegistrationResultVoxelized(
            const Eigen::Matrix4d &transformation = Eigen::Matrix4d::Identity())
        : transformation_(transformation), inlier_rmse_(0.0), fitness_(0.0) {}
    ~RegistrationResultVoxelized() {}


public:
    /// The estimated transformation matrix.
    Eigen::Matrix4d_u transformation_;
    /// Correspondence set between source and target point cloud.
    CorrespondenceSetVoxelized correspondence_set_;
    /// RMSE of all inlier correspondences. Lower is better.
    double inlier_rmse_;
    /// For ICP: the overlapping area (# of inlier correspondences / # of points
    /// in target). Higher is better.
    /// For RANSAC: inlier ratio (# of inlier correspondences / # of
    /// all correspondences)
    double fitness_;
};

/// \brief Functions for ICP registration.
///
/// \param source The source point cloud.
/// \param target The target point cloud.
/// \param max_correspondence_distance Maximum correspondence points-pair
/// distance. \param init Initial transformation estimation.
///  Default value: array([[1., 0., 0., 0.], [0., 1., 0., 0.], [0., 0., 1., 0.],
///  [0., 0., 0., 1.]])
/// \param estimation Estimation method.
/// \param criteria Convergence criteria.
RegistrationResult RegistrationICP(
        const PointCloud &source,
        const PointCloud &target,
        double max_correspondence_distance,
        const Eigen::Matrix4d &init = Eigen::Matrix4d::Identity(),
        const TransformationEstimation &estimation =
                TransformationEstimationPointToPoint(false),
        const ICPConvergenceCriteria &criteria = ICPConvergenceCriteria());


RegistrationResultVoxelized registrationICP(
        const PointCloud &source,
        const VoxelizedPointCloud &target,
        double max_correspondence_distance,
		const TransformationEstimationPointToPlaneVoxelized &estimation,
        const Eigen::Matrix4d &init = Eigen::Matrix4d::Identity(),
		const ICPConvergenceCriteria &criteria = ICPConvergenceCriteria());

}  // namespace open3d_slam
