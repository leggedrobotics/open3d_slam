/*
 * Constraint.hpp
 *
 *  Created on: Nov 9, 2021
 *      Author: jelavice
 */

#pragma once

#include "m545_volumetric_mapping/Transform.hpp"

namespace m545_mapping {

struct Constraint{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Transform sourceToTarget_ = Transform::Identity();
	Transform sourceToTargetPreMultiply_ = Transform::Identity();
	size_t sourceSubmapIdx_=0, targetSubmapIdx_=0;
	Matrix6d informationMatrix_ = Matrix6d::Identity();
	bool isInformationMatrixValid_ = false;
	bool isOdometryConstraint_ = true;
	Time timestamp_;
};

using Constraints = std::vector<Constraint>;

struct OptimizedTransform {
	Transform dT_;
	size_t submapId_;
};

using OptimizedTransforms = std::vector<OptimizedTransform>;

struct OptimizedSubmapPose{
	Transform mapToSubmap_;
	int64 submapId_;
};

using OptimizedSubmapPoses = std::vector<OptimizedSubmapPose>;


} //namespace m545_mapping
