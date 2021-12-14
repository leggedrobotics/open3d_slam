/*
 * OptimizationProblem.hpp
 *
 *  Created on: Nov 12, 2021
 *      Author: jelavice
 */

#pragma once

#include "m545_volumetric_mapping/Constraint.hpp"
#include "m545_volumetric_mapping/Parameters.hpp"
#include <open3d/pipelines/registration/PoseGraph.h>
#include <mutex>

namespace m545_mapping {

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

private:


	bool isRunningOptimization_ = false;
	bool isReadyToOptimize_ = false;
	std::mutex constraintMutex_, optimizationMutex_;
	Constraints loopClosureConstraints_;
	Constraints odometryConstraints_;
	open3d::pipelines::registration::PoseGraph poseGraph_, poseGraphPrev_;
	size_t numLoopClosuresPrev_ = 0;

};

} //namespace m545_mapping
