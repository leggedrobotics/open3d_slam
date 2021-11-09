/*
 * Mapper.hpp
 *
 *  Created on: Sep 26, 2021
 *      Author: jelavice
 */

#pragma once

#include <Eigen/Geometry>
#include <open3d/geometry/PointCloud.h>
#include <open3d/pipelines/registration/Registration.h>
#include "m545_volumetric_mapping/Parameters.hpp"
#include "m545_volumetric_mapping/time.hpp"
#include "m545_volumetric_mapping/croppers.hpp"
#include "m545_volumetric_mapping/Submap.hpp"
#include "m545_volumetric_mapping/TransformInterpolationBuffer.hpp"


namespace m545_mapping {

class Mapper {

public:

	using PointCloud = open3d::geometry::PointCloud;

	Mapper(const TransformInterpolationBuffer &odomToRangeSensorBuffer);
	~Mapper() = default;

	const PointCloud& getMap() const;
	const PointCloud& getDenseMap() const;
	const Submap& getActiveSubmap() const;
	const SubmapCollection &getSubmaps() const;
	PointCloud getAssembledMap() const;
	void addRangeMeasurement(const PointCloud &cloud, const Time &timestamp);
	void setParameters(const MapperParameters &p);
	bool isMatchingInProgress() const;
	bool isManipulatingMap() const;
	bool isReadyForLoopClosure()const;
	void attemptLoopClosures();
	Transform getMapToOdom( const Time &timestamp) const;
	Transform getMapToRangeSensor( const Time &timestamp) const;
	const TransformInterpolationBuffer &getMapToRangeSensorBuffer() const;

private:
	std::shared_ptr<PointCloud> preProcessScan(const PointCloud &scan) const;
	void update(const MapperParameters &p);
	void estimateNormalsIfNeeded(PointCloud *pcl) const;


	bool isMatchingInProgress_ = false;
	bool isManipulatingMap_ = false;
	Time lastMeasurementTimestamp_;
	Transform mapToOdom_ = Transform::Identity();
	Transform odomToRangeSensorPrev_ = Transform::Identity();
	Transform mapToRangeSensor_ = Transform::Identity();
	MapperParameters params_;
	open3d::pipelines::registration::ICPConvergenceCriteria icpCriteria_;
	std::mutex mapManipulationMutex_;
	std::shared_ptr<CroppingVolume> scanMatcherCropper_;
	std::shared_ptr<CroppingVolume> mapBuilderCropper_;
	SubmapCollection submaps_;
	const TransformInterpolationBuffer &odomToRangeSensorBuffer_;
	TransformInterpolationBuffer mapToRangeSensorBuffer_;
	TransformInterpolationBuffer mapToOdomBuffer_;

};

} /* namespace m545_mapping */
