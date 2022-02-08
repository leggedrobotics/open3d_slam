/*
 * OptimizationProblem.hpp
 *
 *  Created on: Nov 12, 2021
 *      Author: jelavice
 */

#pragma once

#include "open3d_slam/Constraint.hpp"
#include "open3d_slam/Parameters.hpp"
#include <open3d/pipelines/registration/PoseGraph.h>
#include <mutex>

namespace o3d_slam {

class SubmapCollection;


class OptimizationProblem {

public:
	OptimizationProblem()=default;

	void clearOdometryConstraints();
	void clearLoopClosureConstraints();
	void addOdometryConstraint(const Constraint &c);
	void addLoopClosureConstraint(const Constraint &c);
	void insertOdometryConstraints(const Constraints &c);
	void insertLoopClosureConstraints(const Constraints &c);
	void solve();
	void buildOptimizationProblem(const SubmapCollection &submaps);
	void setIsReadyToOptimize(bool val);
	bool isRunningOptimization() const;
	void print() const;

	OptimizedTransforms getOptimizedTransformIncrements() const;
	void dumpToFile(const std::string &filename) const;
	void loadFromFile(const std::string &filename);
	void setParameters(const MapperParameters &p);
	const Constraints &getLoopClosureConstraints() const;
	void updateLoopClosureConstraint(size_t idx, const Constraint &c);

private:

	void setupOdometryEdgesAndPoseGraphNodes();
	void setupLoopClosureEdges();

	MapperParameters params_;
	bool isRunningOptimization_ = false;
	bool isReadyToOptimize_ = false;
	std::mutex constraintMutex_, optimizationMutex_;
	Constraints loopClosureConstraints_;
	Constraints odometryConstraints_;
	open3d::pipelines::registration::PoseGraph poseGraph_, poseGraphOptimized_, poseGraphNonOptimized_;
	size_t numLoopClosuresPrev_ = 0;
	size_t numOdometryEdgesPrev_ = 0;

};

} //namespace o3d_slam
