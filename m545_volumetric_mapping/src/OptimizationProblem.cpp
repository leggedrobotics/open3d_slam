/*
 * OptimizationProblem.cpp
 *
 *  Created on: Nov 12, 2021
 *      Author: jelavice
 */

#include "m545_volumetric_mapping/OptimizationProblem.hpp"
#include "m545_volumetric_mapping/Submap.hpp"
#include "m545_volumetric_mapping/SubmapCollection.hpp"
#include <open3d/pipelines/registration/GlobalOptimization.h>


namespace m545_mapping {

namespace {
namespace registration = open3d::pipelines::registration;
} //namespace

void OptimizationProblem::addNodes(const SubmapCollection &submaps) {


	for (const auto &submap : submaps.getSubmaps()) {
		registration::PoseGraphNode node;
		node.pose_ = submap.getMapToSubmapOrigin().matrix();
		poseGraph_.nodes_.emplace_back(std::move(node));
	}

}

void OptimizationProblem::buildOptimizationProblem() {

	registration::PoseGraph poseGraph;

	for (const auto &c : odometryConstraints_) {
		registration::PoseGraphEdge edge;
		edge.source_node_id_ = c.sourceSubmapIdx_;
		edge.target_node_id_ = c.targetSubmapIdx_;
		edge.transformation_ = c.sourceToTarget_.matrix();
		edge.uncertain_ = false;
		poseGraph_.edges_.push_back(std::move(edge));
	}

	for (const auto &c : loopClosureConstraints_) {
		registration::PoseGraphEdge edge;
		edge.source_node_id_ = c.sourceSubmapIdx_;
		edge.target_node_id_ = c.targetSubmapIdx_;
		edge.transformation_ = c.sourceToTarget_.matrix();
		edge.uncertain_ = true;
		poseGraph_.edges_.push_back(std::move(edge));
	}

	GlobalOptimization(poseGraph_);

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
