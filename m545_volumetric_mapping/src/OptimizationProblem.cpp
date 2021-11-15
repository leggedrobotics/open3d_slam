/*
 * OptimizationProblem.cpp
 *
 *  Created on: Nov 12, 2021
 *      Author: jelavice
 */

#include "m545_volumetric_mapping/OptimizationProblem.hpp"
#include "m545_volumetric_mapping/Submap.hpp"
#include "m545_volumetric_mapping/SubmapCollection.hpp"

namespace m545_mapping {

namespace {
namespace registration = open3d::pipelines::registration;
} //namespace

void OptimizationProblem::addNodes(const SubmapCollection &submaps) {

	nodes_.clear();
	nodes_.reserve(submaps.getSubmaps().size());
	for (const auto &submap : submaps.getSubmaps()) {
		registration::PoseGraphNode node;
		node.pose_ = submap.getMapToSubmapOrigin().matrix();
		nodes_.emplace_back(std::move(node));
	}

}

void OptimizationProblem::clearOdometryConstraints() {
	odometryConstraints_.clear();
}
void OptimizationProblem::clearLoopClosureConstraints() {
	loopClosureConstraints_.clear();
}

void OptimizationProblem::addOdometryConstraint(const Constraint &c) {
	odometryConstraints_.push_back(c);
}

void OptimizationProblem::addLoopClosureConstraint(const Constraint &c) {
	loopClosureConstraints_.push_back(c);
}

} //namespace m545_mapping
