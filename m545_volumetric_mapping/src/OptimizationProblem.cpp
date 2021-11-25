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
#include <open3d/io/PoseGraphIO.h>

#include <fstream>

namespace m545_mapping {

namespace {
namespace registration = open3d::pipelines::registration;
} //namespace

void OptimizationProblem::solve() {
	const Timer t("global_optimization");
	std::lock_guard<std::mutex> lck(optimizationMutex_);
	isRunningOptimization_ = true;
	isReadyToOptimize_ = false;
	std::cout << "Optimizing graph...\n";
	registration::GlobalOptimizationLevenbergMarquardt method;
	registration::GlobalOptimizationConvergenceCriteria criteria;
	registration::GlobalOptimizationOption option;
	option.max_correspondence_distance_ = 3.0;
	option.reference_node_ = 0;
	option.edge_prune_threshold_ = 0.05;
	option.preference_loop_closure_ = 1.0;
	GlobalOptimization(poseGraph_, method, criteria, option);
	isRunningOptimization_ = false;
}

void OptimizationProblem::buildOptimizationProblem(const SubmapCollection &submaps) {
	std::lock_guard<std::mutex> lck(optimizationMutex_);
	std::lock_guard<std::mutex> lck2(constraintMutex_);
	isRunningOptimization_ = true;
	isReadyToOptimize_ = false;

	poseGraph_.nodes_.clear();
	for (const auto &submap : submaps.getSubmaps()) {
		registration::PoseGraphNode node;
		node.pose_ = submap.getMapToSubmapOrigin().matrix();
		poseGraph_.nodes_.emplace_back(std::move(node));
	}

	poseGraph_.edges_.clear();

	for (const auto &c : odometryConstraints_) {
		registration::PoseGraphEdge edge;
		edge.source_node_id_ = c.sourceSubmapIdx_;
		edge.target_node_id_ = c.targetSubmapIdx_;
		edge.transformation_ = c.sourceToTarget_.inverse().matrix();
		edge.information_ = c.informationMatrix_;
		edge.uncertain_ = false;
		poseGraph_.edges_.push_back(std::move(edge));
	}
	numLoopClosuresPrev_ = loopClosureConstraints_.size();
	for (const auto &c : loopClosureConstraints_) {
		registration::PoseGraphEdge edge;
		edge.source_node_id_ = c.sourceSubmapIdx_;
		edge.target_node_id_ = c.targetSubmapIdx_;
		edge.transformation_ = c.sourceToTarget_.inverse().matrix();
		edge.information_ = c.informationMatrix_;
		assert_true(c.isInformationMatrixValid_,
				"Invalid information matrix between: " + std::to_string(c.sourceSubmapIdx_) + " and "
						+ std::to_string(c.targetSubmapIdx_));
		assert_gt(c.sourceSubmapIdx_, c.targetSubmapIdx_);

		edge.uncertain_ = true;
		poseGraph_.edges_.push_back(std::move(edge));
	}
	poseGraphPrev_ = poseGraph_;
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

void OptimizationProblem::dumpToFile(const std::string &filename) const {
	open3d::io::WritePoseGraph(filename, poseGraph_);
}

OptimizedSubmapPoses OptimizationProblem::getOptimizedNodeValues() const {
	return getNodeValues(poseGraph_);
}

OptimizedSubmapPoses OptimizationProblem::getNodeValues(const registration::PoseGraph &poseGraph) const {
	OptimizedSubmapPoses retVal;
	retVal.reserve(poseGraph.nodes_.size());
	for (size_t i = 0; i < poseGraph.nodes_.size(); ++i) {
		OptimizedSubmapPose p;
		p.mapToSubmap_ = Transform(poseGraph.nodes_.at(i).pose_);
		p.submapId_ = i;
		retVal.emplace_back(std::move(p));
	}
	return retVal;
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

void OptimizationProblem::insertOdometryConstraints(const Constraints &c) {
	std::lock_guard<std::mutex> lck(constraintMutex_);
	odometryConstraints_.insert(odometryConstraints_.end(),c.begin(),c.end());
}

void OptimizationProblem::insertLoopClosureConstraints(const Constraints &c) {
	std::lock_guard<std::mutex> lck(constraintMutex_);
	loopClosureConstraints_.insert(loopClosureConstraints_.end(),c.begin(),c.end());
}


OptimizedTransforms OptimizationProblem::getOptimizedTransformIncrements() const {
	OptimizedTransforms retVal;
	const auto nonOptimizedPoses = getNodeValues(poseGraphPrev_);
	const auto optimizedPoses = getNodeValues(poseGraph_);
	assert_eq(nonOptimizedPoses.size(), optimizedPoses.size(),
			"Graphs are not of same size, did you run the optimization?");
	for (size_t i = 0; i < optimizedPoses.size(); ++i) {
		const auto mapToOld = nonOptimizedPoses.at(i).mapToSubmap_;
		const auto mapToNew = optimizedPoses.at(i).mapToSubmap_;
		const auto deltaT = mapToOld.inverse() * mapToNew;
		retVal.emplace_back(OptimizedTransform{ deltaT, nonOptimizedPoses.at(i).submapId_ });
	}

	return retVal;
}

} //namespace m545_mapping
