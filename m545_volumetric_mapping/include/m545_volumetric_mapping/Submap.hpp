/*
 * Submap.hpp
 *
 *  Created on: Oct 27, 2021
 *      Author: jelavice
 */

#pragma once

#include <open3d/geometry/PointCloud.h>
#include <ros/time.h>
#include <Eigen/Dense>
#include <mutex>
#include "m545_volumetric_mapping/Parameters.hpp"
#include "m545_volumetric_mapping/croppers.hpp"
#include "m545_volumetric_mapping/time.hpp"
#include "m545_volumetric_mapping/Transform.hpp"
#include <open3d/pipelines/registration/Feature.h>
#include "m545_volumetric_mapping/Constraint.hpp"
namespace m545_mapping {

class Submap {

public:
	using PointCloud = open3d::geometry::PointCloud;
	using Feature = open3d::pipelines::registration::Feature;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

	Submap();
	~Submap() = default;

	void setParameters(const MapperParameters &mapperParams);
	bool insertScan(const PointCloud &rawScan, const PointCloud &preProcessedScan, const Transform &transform, const Time &time);
	void voxelizeInsideCroppingVolume(const CroppingVolume &cropper, const MapBuilderParameters &param,
			PointCloud *map) const;
	const Transform& getMapToSubmap() const;
	const PointCloud& getMap() const;
	const PointCloud& getDenseMap() const;
	void setMapToSubmap(const Transform &T);
	bool isEmpty() const;
	void computeFeatures();
	const Feature& getFeatures() const;
	const PointCloud& getSparseMap() const;
	void centerOrigin();
	Time getLastScanInsertionTime() const;
	mutable PointCloud toRemove_;
	mutable PointCloud scanRef_;
	mutable PointCloud mapRef_;

private:
	void update(const MapperParameters &mapperParams);
	void estimateNormalsIfNeeded(int knn, PointCloud *pcl) const;
	void carve(const PointCloud &rawScan, const Transform &mapToRangeSensor, const CroppingVolume &cropper,
			const SpaceCarvingParameters &params, PointCloud *map, Timer *timer) const;

	PointCloud sparseMap_;
	PointCloud map_;
	PointCloud denseMap_;
	Transform mapToSubmap_ = Transform::Identity();
	Transform mapToRangeSensor_ = Transform::Identity();
	std::shared_ptr<CroppingVolume> mapBuilderCropper_;
	std::shared_ptr<CroppingVolume> denseMapCropper_;
	MapperParameters params_;
	Timer carvingTimer_;
	Timer carveDenseMapTimer_;
	std::shared_ptr<Feature> feature_;
	Time lastScanInsertionTime_;
};

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
	void updateActiveSubmap(const Transform &mapToRangeSensor);
private:

	void updateActiveSubmap();
	void createNewSubmap(const Transform &mapToSubmap);
	size_t findClosestSubmap(const Transform &mapToRangesensor) const;
	std::vector<size_t> getCloseSubmapsIdxs(const Transform &mapToRangeSensor, size_t lastFinishedSubmapIdx,
			size_t currentActiveSubmapIdx) const;

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
	std::vector<Constraint> constraints_;

};

} // namespace m545_mapping
