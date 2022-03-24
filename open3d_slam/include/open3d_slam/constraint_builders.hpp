/*
 * constraint_builders.hpp
 *
 *  Created on: Feb 20, 2022
 *      Author: jelavice
 */


#pragma once

#include <Eigen/Dense>
#include "open3d_slam/SubmapCollection.hpp"
#include "open3d_slam/Constraint.hpp"



namespace o3d_slam {

void computeOdometryConstraints(const SubmapCollection &submaps,
		Constraints *constraints);
void computeOdometryConstraints(const SubmapCollection &submaps, const SubmapCollection::TimestampedSubmapIds &candidates,
		Constraints *constraints);

Constraint buildOdometryConstraint(size_t sourceIdx, size_t targetIdx,
		const SubmapCollection &submaps);
Constraint buildConstraint(size_t sourceIdx, size_t targetIdx,
		const SubmapCollection &submaps, bool isComputeOverlap, double icpMaxCorrespondenceDistance, double voxelSizeOverlapCompute, bool isEstimateInformationMatrix, bool isSkipIcpRefinement);

} // namespace o3d_slam
