/*
 * SubmapCollection.hpp
 *
 *  Created on: Nov 10, 2021
 *      Author: jelavice
 */


#pragma once

#include <open3d/geometry/PointCloud.h>
#include <ros/time.h>
#include <Eigen/Dense>
#include <mutex>
#include "m545_volumetric_mapping/Parameters.hpp"
#include "m545_volumetric_mapping/croppers.hpp"
#include "m545_volumetric_mapping/Submap.hpp"
#include "m545_volumetric_mapping/Constraint.hpp"
#include "m545_volumetric_mapping/AdjacencyMatrix.hpp"
#include "m545_volumetric_mapping/PlaceRecognition.hpp"
#include "m545_volumetric_mapping/OptimizationProblem.hpp"


namespace m545_mapping {

class SubmapCollection {
public:
	using Submaps = std::vector<Submap>;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	SubmapCollection();
	~SubmapCollection() = default;

	using PointCloud = open3d::geometry::PointCloud;
	void setMapToRangeSensor(const Transform &T);
	const Submap& getActiveSubmap() const;
	bool insertScan(const PointCloud &rawScan, const PointCloud &preProcessedScan,
			const Transform &mapToRangeSensor, const Time &timestamp);
	void setParameters(const MapperParameters &p);
	bool isEmpty() const;
	const Submaps& getSubmaps() const;
	size_t getTotalNumPoints() const;
	void computeFeaturesInLastFinishedSubmap();
	bool isFinishedSubmap() const;
	void buildLoopClosureConstraints();
	bool isBuildingLoopClosureConstraints() const;
	const std::vector<Constraint> &getConstraints() const;
	void clearConstraints();
	std::vector<Constraint> getAndClearConstraints();
private:

	void updateActiveSubmap(const Transform &mapToRangeSensor);
	void createNewSubmap(const Transform &mapToSubmap);
	size_t findClosestSubmap(const Transform &mapToRangesensor) const;
	Constraint buildOdometryConstraint(size_t sourceSubmapIdx, size_t targetSubmapIdx) const;

	Transform mapToRangeSensor_ = Transform::Identity();
	std::vector<Submap> submaps_;
	size_t activeSubmapIdx_ = 0;
	MapperParameters params_;
	size_t numScansMergedInActiveSubmap_ = 0;
	bool isFinishedSubmap_ = false;
	bool isBuildingLoopClosureConstraints_ = false;
	size_t lastFinishedSubmapIdx_ = 0;
	std::mutex featureComputationMutex_;
	std::mutex loopClosureConstraintMutex_;
	std::mutex constraintBuildMutex_;
	Constraints constraints_;
	AdjacencyMatrix adjacencyMatrix_;
	size_t submapId_=0;
	PlaceRecognition placeRecognition_;
	OptimizationProblem optimization_;

};

} // namespace m545_mapping
