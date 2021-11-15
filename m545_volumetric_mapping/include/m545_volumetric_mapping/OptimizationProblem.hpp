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
#include <open3d/pipelines/registration/GlobalOptimization.h>


namespace m545_mapping {

class Submap;
class AdjacencyMatrix;
class SubmapCollection;


class OptimizationProblem {

public:
	OptimizationProblem()=default;

	void clearOdometryConstraints();
	void clearLoopClosureConstraints();
	void addOdometryConstraint(const Constraint &c);
	void addLoopClosureConstraint(const Constraint &c);
	void addNodes(const SubmapCollection &submaps);

private:
	Constraints loopClosureConstraints_;
	Constraints odometryConstraints_;
	std::vector<open3d::pipelines::registration::PoseGraphNode> nodes_;
	std::vector<open3d::pipelines::registration::PoseGraphEdge> edges_;

};

} //namespace m545_mapping
