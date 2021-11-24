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

namespace m545_mapping {

class Submap {

public:
	using PointCloud = open3d::geometry::PointCloud;
	using Feature = open3d::pipelines::registration::Feature;
	using SubmapId = int64;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

	Submap(size_t id);
	~Submap() = default;

	void setParameters(const MapperParameters &mapperParams);
	bool insertScan(const PointCloud &rawScan, const PointCloud &preProcessedScan, const Transform &transform, const Time &time, bool isPerformCarving);

	const Transform& getMapToSubmapOrigin() const;
	Eigen::Vector3d getMapToSubmapCenter() const;
	void setMapToSubmapOrigin(const Transform &T);
	const PointCloud& getMap() const;
	const PointCloud& getDenseMap() const;
	bool isEmpty() const;
	const Feature& getFeatures() const;
	const PointCloud& getSparseMap() const;
	void computeSubmapCenter();
	void computeFeatures();
	Feature *getFeaturePtr() const;
	Time getCreationTime() const;
	SubmapId getId() const;
	void transform(const Transform &T);
	mutable PointCloud toRemove_;
	mutable PointCloud scanRef_;
	mutable PointCloud mapRef_;

private:
	void update(const MapperParameters &mapperParams);
	void estimateNormalsIfNeeded(int knn, PointCloud *pcl) const;
	void carve(const PointCloud &rawScan, const Transform &mapToRangeSensor, const CroppingVolume &cropper,
			const SpaceCarvingParameters &params, PointCloud *map, Timer *timer) const;
	void voxelizeInsideCroppingVolume(const CroppingVolume &cropper, const MapBuilderParameters &param,
			PointCloud *map) const;

	PointCloud sparseMap_, map_, denseMap_;
	Transform mapToSubmap_ = Transform::Identity();
	Transform mapToRangeSensor_ = Transform::Identity();
	Eigen::Vector3d submapCenter_ = Eigen::Vector3d::Zero();
	std::shared_ptr<CroppingVolume> denseMapCropper_,mapBuilderCropper_;
	MapperParameters params_;
	Timer carveDenseMapTimer_, carvingTimer_, featureTimer_;
	std::shared_ptr<Feature> feature_;
	Time creationTime_;
	size_t id_=0;
	bool isCenterComputed_ = false;

};

} // namespace m545_mapping
