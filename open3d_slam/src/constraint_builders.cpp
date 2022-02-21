/*
 * constraint_builders.cpp
 *
 *  Created on: Feb 20, 2022
 *      Author: jelavice
 */






#include <Eigen/Dense>
#include "open3d_slam/constraint_builders.hpp"
#include "open3d_slam/typedefs.hpp"
#include "open3d_slam/math.hpp"
#include "open3d_slam/magic.hpp"
#include <open3d/pipelines/registration/Registration.h>
#include "open3d_slam/helpers.hpp"


namespace o3d_slam {

////////////////////////////////////////////////////////////////////
/// NON MEMBER ////////
////////////////////////////////////////////////////////////////////
namespace {
bool hasConstraint(size_t sourceIdx, size_t targetIdx, const Constraints &constraints) {
	for (const auto &c : constraints) {
		if (c.sourceSubmapIdx_ == sourceIdx && c.targetSubmapIdx_ == targetIdx) {
			return true;
		}
	}
	return false;
}
} // namespace

Constraint buildOdometryConstraint(size_t sourceIdx, size_t targetIdx, const SubmapCollection &submaps) {
	const double mapVoxelSize = getMapVoxelSize(submaps.getParameters().mapBuilder_,
			voxelSizeCorrespondenceSearchMapVoxelSizeIsZero);
	Constraint c = buildConstraint(sourceIdx, targetIdx, submaps, true, 1.5 * mapVoxelSize,
			20.0 * mapVoxelSize,true);
	c.isOdometryConstraint_ = true;
	return c;
}

Constraint buildConstraint(size_t sourceIdx, size_t targetIdx, const SubmapCollection &submaps,
		bool isComputeOverlap, double icpMaxCorrespondenceDistance, double voxelSizeOverlapCompute,
		bool isEstimateInformationMatrix) {

	PointCloud source = submaps.getSubmaps().at(sourceIdx).getMap();
	PointCloud target = submaps.getSubmaps().at(targetIdx).getMap();
	const double mapVoxelSize = getMapVoxelSize(submaps.getParameters().mapBuilder_,
			voxelSizeCorrespondenceSearchMapVoxelSizeIsZero);

	if (isComputeOverlap) {
		std::vector<size_t> sourceIdxs, targetIdxs;
		const size_t minNumPointsPerVoxel = 1;
		computeIndicesOfOverlappingPoints(source, target, Transform::Identity(), voxelSizeOverlapCompute,
				minNumPointsPerVoxel, &sourceIdxs, &targetIdxs);
		source = *source.SelectByIndex(sourceIdxs);
		target = *target.SelectByIndex(targetIdxs);
	}

	open3d::pipelines::registration::ICPConvergenceCriteria criteria;
	criteria.max_iteration_ = icpRunUntilConvergenceNumberOfIterations; // i.e. run until convergence
	const auto icpResult = open3d::pipelines::registration::RegistrationICP(source, target,
			icpMaxCorrespondenceDistance, Eigen::Matrix4d::Identity(),
			open3d::pipelines::registration::TransformationEstimationPointToPlane(), criteria);

	Eigen::Matrix6d informationMatrix = Eigen::Matrix6d::Identity();
	if (isEstimateInformationMatrix) {
		informationMatrix = open3d::pipelines::registration::GetInformationMatrixFromPointClouds(source, target,
				icpMaxCorrespondenceDistance, icpResult.transformation_);
	}

	Constraint c;
	c.sourceSubmapIdx_ = sourceIdx;
	c.targetSubmapIdx_ = targetIdx;
	c.isOdometryConstraint_ = true;
	c.isInformationMatrixValid_ = isEstimateInformationMatrix;
	c.sourceToTarget_ = Transform(icpResult.transformation_);
//	c.sourceToTarget_ = Transform::Identity();
	c.informationMatrix_ = informationMatrix;

//	printf("submap %ld size: %ld \n", sourceIdx, sourceFull.points_.size());
//	printf("submap %ld overlap size: %ld \n", sourceIdx, source.points_.size());
//	printf("submap %ld size: %ld \n", targetIdx, targetFull.points_.size());
//	printf("submap %ld overlap size: %ld \n", targetIdx, target.points_.size());
//	printf("Adding odometry constraint from submap %ld to submap %ld with transformation \n: ",
//			sourceIdx, targetIdx);
//	std::cout << asString(c.sourceToTarget_) << std::endl;
	return c;

}

void computeOdometryConstraints(const SubmapCollection &submaps,
		const SubmapCollection::TimestampedSubmapIds &candidates, Constraints *constraints) {
	const size_t nSubmaps = submaps.getSubmaps().size();
	const size_t activeSubmapIdx = submaps.getActiveSubmap().getId();
	for (const auto candidate : candidates) {
		if (candidate.submapId_ < 1) {
			continue;
		}
		const size_t targetCandidate = candidate.submapId_;
		const size_t sourceCandidate = submaps.getSubmaps().at(targetCandidate).getParentId();
		if (!hasConstraint(sourceCandidate, targetCandidate, *constraints)) {
			const Constraint c = buildOdometryConstraint(sourceCandidate, targetCandidate, submaps);
			constraints->emplace_back(std::move(c));
		}

	}

}

void computeOdometryConstraints(const SubmapCollection &submaps, Constraints *constraints) {
	const size_t nSubmaps = submaps.getSubmaps().size();
	const size_t activeSubmapIdx = submaps.getActiveSubmap().getId();
	for (size_t submapIdx = 1; submapIdx < nSubmaps; ++submapIdx) {
		const size_t targetIdx = submapIdx;
		const size_t sourceIdx = submaps.getSubmaps().at(targetIdx).getParentId();
		if (!hasConstraint(sourceIdx, targetIdx, *constraints) && sourceIdx != activeSubmapIdx
				&& targetIdx != activeSubmapIdx) {
			const Constraint c = buildOdometryConstraint(sourceIdx, targetIdx, submaps);
			constraints->emplace_back(std::move(c));
		}
	}

}
} // namespace o3d_slam