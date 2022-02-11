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
#include "open3d_slam/Parameters.hpp"
#include "open3d_slam/time.hpp"
#include "open3d_slam/croppers.hpp"
#include "open3d_slam/SubmapCollection.hpp"
#include "open3d_slam/TransformInterpolationBuffer.hpp"

namespace o3d_slam {

class Mapper {

public:

	using PointCloud = open3d::geometry::PointCloud;

	Mapper(const TransformInterpolationBuffer &odomToRangeSensorBuffer,
			std::shared_ptr<SubmapCollection> submaps);
	~Mapper() = default;

	void setMapToRangeSensor(const Transform &t);
	const PointCloud& getMap() const;
	const PointCloud& getDenseMap() const;
	const Submap& getActiveSubmap() const;
	const SubmapCollection& getSubmaps() const;
	SubmapCollection* getSubmapsPtr();
	PointCloud getAssembledMap() const;
	bool addRangeMeasurement(const PointCloud &cloud, const Time &timestamp);
	void setParameters(const MapperParameters &p);
	bool isMatchingInProgress() const;
	bool isManipulatingMap() const;
	Transform getMapToOdom(const Time &timestamp) const;
	Transform getMapToRangeSensor(const Time &timestamp) const;
	const TransformInterpolationBuffer& getMapToRangeSensorBuffer() const;
	TransformInterpolationBuffer* getMapToRangeSensorBufferPtr();
	const PointCloud& getPreprocessedScan() const;
	void loopClosureUpdate(const Transform &loopClosureCorrection);
private:
	std::shared_ptr<PointCloud> preProcessScan(const PointCloud &scan) const;
	void update(const MapperParameters &p);
	void estimateNormalsIfNeeded(PointCloud *pcl) const;
	void checkTransformChainingAndPrintResult(bool isCheckTransformChainingAndPrintResult) const;

	bool isMatchingInProgress_ = false;
	bool isManipulatingMap_ = false;
	Time lastMeasurementTimestamp_;
	Transform mapToRangeSensor_ = Transform::Identity();
	Transform mapToRangeSensorPrev_ = Transform::Identity();
	Transform mapToRangeSensorLastScanInsertion_ = Transform::Identity();

	MapperParameters params_;
	open3d::pipelines::registration::ICPConvergenceCriteria icpCriteria_;
	std::mutex mapManipulationMutex_;
	std::shared_ptr<CroppingVolume> scanMatcherCropper_;
	std::shared_ptr<CroppingVolume> mapBuilderCropper_;
	std::shared_ptr<SubmapCollection> submaps_;
	const TransformInterpolationBuffer &odomToRangeSensorBuffer_;
	TransformInterpolationBuffer mapToRangeSensorBuffer_;
	open3d::geometry::PointCloud preProcessedScan_;

};

} /* namespace o3d_slam */
