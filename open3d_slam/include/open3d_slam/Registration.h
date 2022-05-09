
#pragma once

#include <Eigen/Core>
#include <tuple>
#include <vector>

#include "open3d/pipelines/registration/Registration.h"
#include "open3d_slam/typedefs.hpp"
#include "open3d_slam/TransformationEstimation.h"

namespace o3d_slam {

using ICPConvergenceCriteria = open3d::pipelines::registration::ICPConvergenceCriteria;
using CorrespondenceSet = open3d::pipelines::registration::CorrespondenceSet;
using RegistrationResult = open3d::pipelines::registration::RegistrationResult;

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


}  // namespace open3d_slam
