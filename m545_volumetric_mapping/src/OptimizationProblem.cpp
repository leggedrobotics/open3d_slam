/*
 * OptimizationProblem.cpp
 *
 *  Created on: Nov 12, 2021
 *      Author: jelavice
 */

#include "m545_volumetric_mapping/OptimizationProblem.hpp"
#include "m545_volumetric_mapping/Submap.hpp"
#include "m545_volumetric_mapping/assert.hpp"
#include "m545_volumetric_mapping/helpers.hpp"

#include "m545_volumetric_mapping/SubmapCollection.hpp"
#include <open3d/pipelines/registration/GlobalOptimization.h>

namespace m545_mapping {

namespace {
namespace registration = open3d::pipelines::registration;
} //namespace

void OptimizationProblem::solve(const SubmapCollection &submaps) {
	const Timer t("global_optimization");
	std::lock_guard<std::mutex> lck(optimizationMutex_);
	isRunningOptimization_ = true;
	isReadyToOptimize_ = false;

	poseGraph_.nodes_.clear();
	for (const auto &submap : submaps.getSubmaps()) {
		registration::PoseGraphNode node;
		node.pose_ = submap.getMapToSubmapOrigin().matrix();
		poseGraph_.nodes_.emplace_back(std::move(node));
	}

	{
		std::lock_guard<std::mutex> lck2(constraintMutex_);
		poseGraph_.edges_.clear();

		for (const auto &c : odometryConstraints_) {
			registration::PoseGraphEdge edge;
			edge.source_node_id_ = c.sourceSubmapIdx_;
			edge.target_node_id_ = c.targetSubmapIdx_;
			edge.transformation_ = c.sourceToTarget_.matrix();
			edge.uncertain_ = false;
			poseGraph_.edges_.push_back(std::move(edge));
		}
		numLoopClosuresPrev_ = loopClosureConstraints_.size();
		for (const auto &c : loopClosureConstraints_) {
			registration::PoseGraphEdge edge;
			edge.source_node_id_ = c.sourceSubmapIdx_;
			edge.target_node_id_ = c.targetSubmapIdx_;
			edge.transformation_ = c.sourceToTarget_.matrix();
			edge.uncertain_ = true;
			poseGraph_.edges_.push_back(std::move(edge));
		}
	}
	print();
	std::cout << "Optimizing graph...\n";
	GlobalOptimization(poseGraph_);
	isRunningOptimization_ = false;
}

bool OptimizationProblem::isReadyToOptimize() const {
	assert_ge(loopClosureConstraints_.size(), numLoopClosuresPrev_);
	return numLoopClosuresPrev_ < loopClosureConstraints_.size();
}

void OptimizationProblem::print() const {
	const auto graph = poseGraph_; // copy
	const size_t nNodes = graph.nodes_.size();
	const size_t nEdges = graph.edges_.size();
	std::cout << "The problem contains: " << nNodes << " nodes and " << nEdges << " edges \n";
	for (int i = 0; i < nEdges; ++i) {
		const auto &e = graph.edges_.at(i);
		std::cout << " edge " << i << " from " << e.source_node_id_ << " to " << e.target_node_id_ << std::endl;
	}
	for (int i = 0; i < nNodes; ++i) {
		const auto &n = graph.nodes_.at(i);
		const Transform T(n.pose_);
		std::cout << " node " << i << " pose: " << asString(T) << " \n ";
	}

}

void OptimizationProblem::clearOdometryConstraints() {
	odometryConstraints_.clear();
}
void OptimizationProblem::clearLoopClosureConstraints() {
	loopClosureConstraints_.clear();
}

bool OptimizationProblem::isRunningOptimization() const {
	return isRunningOptimization_;
}

void OptimizationProblem::addOdometryConstraint(const Constraint &c) {
	std::lock_guard<std::mutex> lck(constraintMutex_);
	odometryConstraints_.push_back(c);
}

void OptimizationProblem::addLoopClosureConstraint(const Constraint &c) {
	std::lock_guard<std::mutex> lck(constraintMutex_);
	loopClosureConstraints_.push_back(c);
}

} //namespace m545_mapping
