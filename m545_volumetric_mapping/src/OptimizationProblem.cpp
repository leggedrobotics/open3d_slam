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
	const auto &p = params_.globalOptimization_;
	option.max_correspondence_distance_ = p.maxCorrespondenceDistance_;
	option.reference_node_ = p.referenceNode_;
	option.edge_prune_threshold_ = p.edgePruneThreshold_;
	option.preference_loop_closure_ = p.loopClosurePreference_;
	GlobalOptimization(poseGraph_, method, criteria, option);
	isRunningOptimization_ = false;
}

void OptimizationProblem::setParameters(const MapperParameters &p) {
	params_ = p;
}

void OptimizationProblem::buildOptimizationProblem(const SubmapCollection &submaps) {
	std::lock_guard<std::mutex> lck(optimizationMutex_);
	std::lock_guard<std::mutex> lck2(constraintMutex_);
	isRunningOptimization_ = true;
	isReadyToOptimize_ = false;

	poseGraph_.nodes_.clear();
	poseGraph_.edges_.clear();
	registration::PoseGraphNode prototypeNode;
	prototypeNode.pose_ = Eigen::Matrix4d::Identity();
	poseGraph_.nodes_.push_back(prototypeNode);
	Eigen::Matrix4d odometry = Eigen::Matrix4d::Identity();

	//ensure that odometry constraint sources are in increasing order
	std::sort(odometryConstraints_.begin(), odometryConstraints_.end(),
			[](const Constraint &c1, const Constraint &c2) {
				return c1.sourceSubmapIdx_ < c2.targetSubmapIdx_;
			});

	for (const auto &c : odometryConstraints_) {
		odometry = odometry * c.sourceToTarget_.matrix();
		prototypeNode.pose_ = odometry;
//		std::cout << "odom: " << asString(Transform(prototypeNode.pose_)) << "\n";
		poseGraph_.nodes_.push_back(prototypeNode);
		registration::PoseGraphEdge edge;
		edge.source_node_id_ = c.sourceSubmapIdx_;
		edge.target_node_id_ = c.targetSubmapIdx_;
		edge.transformation_ = c.sourceToTarget_.matrix();
		edge.information_ = c.informationMatrix_;
		edge.uncertain_ = false;
		poseGraph_.edges_.push_back(std::move(edge));
	}
	numLoopClosuresPrev_ = loopClosureConstraints_.size();
	for (const auto &c : loopClosureConstraints_) {
		registration::PoseGraphEdge edge;
		edge.source_node_id_ = c.sourceSubmapIdx_;
		edge.target_node_id_ = c.targetSubmapIdx_;
		edge.transformation_ = c.sourceToTarget_.matrix();
		edge.information_ = c.informationMatrix_;
		assert_true(c.isInformationMatrixValid_,
				"Invalid information matrix between: " + std::to_string(c.sourceSubmapIdx_) + " and "
						+ std::to_string(c.targetSubmapIdx_));
		assert_gt(c.sourceSubmapIdx_, c.targetSubmapIdx_, "Optimization problem, loop closure constraints: ");
		edge.uncertain_ = true;
		poseGraph_.edges_.push_back(std::move(edge));
	}
	poseGraphPrev_ = poseGraph_;
//	loopClosureConstraints_.clear();

	//this is an approxmiation

	//iterate over the loopClosure constraints and set the transform to identity
	// this assumes that they are ideally aligned
	// todo realign with ICP
	// re - estimate the information matrix

	for (auto &loopClosingConstraint : loopClosureConstraints_) {
//		loopClosingConstraint.sourceToTarget_.setIdentity();
		std::cout << " loop closure from submap: " << loopClosingConstraint.sourceSubmapIdx_ << " to submap "
				<< loopClosingConstraint.targetSubmapIdx_ << "\n";
	}

	isRunningOptimization_ = false;
}

void OptimizationProblem::print() const {
	const auto graph = poseGraph_; // copy
	const size_t nNodes = graph.nodes_.size();
	const size_t nEdges = graph.edges_.size();
	std::cout << "The problem contains: " << nNodes << " nodes and " << nEdges << " edges \n";

	for (int i = 0; i < nNodes; ++i) {
		const auto &n = graph.nodes_.at(i);
		const Transform T(n.pose_);
		std::cout << " node " << i << " pose: " << asString(T) << " \n ";
	}

	for (int i = 0; i < nEdges; ++i) {
		const auto &e = graph.edges_.at(i);
		const Transform T(e.transformation_);
		std::cout << " edge " << i << " from " << e.source_node_id_ << " to " << e.target_node_id_ << " with "
				<< asString(T) << std::endl;
	}

}

void OptimizationProblem::dumpToFile(const std::string &filename) const {
	open3d::io::WritePoseGraph(filename, poseGraph_);
}

void OptimizationProblem::loadFromFile(const std::string &filename) {
	open3d::io::ReadPoseGraph(filename, poseGraph_);
	poseGraphPrev_ = poseGraph_;
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
	odometryConstraints_.insert(odometryConstraints_.end(), c.begin(), c.end());
}

void OptimizationProblem::insertLoopClosureConstraints(const Constraints &cs) {
	std::lock_guard<std::mutex> lck(constraintMutex_);
	for (const auto &c : cs) {
		auto hasConstraintAlready = [&c](const Constraint &c2) -> bool {
			return c.sourceSubmapIdx_ == c2.sourceSubmapIdx_ && c.targetSubmapIdx_ == c2.targetSubmapIdx_;
		};
		const auto search = std::find_if(loopClosureConstraints_.begin(), loopClosureConstraints_.end(),
				hasConstraintAlready);
		const bool isConstraintAlreadyExists = search != loopClosureConstraints_.end();
		if (!isConstraintAlreadyExists){
			loopClosureConstraints_.push_back(c);
		}
	}
//	loopClosureConstraints_.insert(loopClosureConstraints_.end(), cs.begin(), cs.end());
}

OptimizedTransforms OptimizationProblem::getOptimizedTransformIncrements() const {
	OptimizedTransforms retVal;
	assert_eq(poseGraphPrev_.nodes_.size(), poseGraph_.nodes_.size(),
			"Graphs are not of same size, did you run the optimization?");
	for (size_t i = 0; i < poseGraph_.nodes_.size(); ++i) {
		const auto deltaT = Transform(poseGraph_.nodes_.at(i).pose_);
		retVal.emplace_back(OptimizedTransform { deltaT, i });
	}

	return retVal;
}

} //namespace m545_mapping
