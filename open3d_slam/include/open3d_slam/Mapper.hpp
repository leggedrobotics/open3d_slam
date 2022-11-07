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

class ScanToMapRegistration;

class Mapper {

public:

	using PointCloud = open3d::geometry::PointCloud;

	Mapper(const TransformInterpolationBuffer &odomToRangeSensorBuffer,
			std::shared_ptr<SubmapCollection> submaps);
	~Mapper() = default;

	void setParameters(const MapperParameters &p);
	void setMapToRangeSensor(const Transform &t);
	void setMapToRangeSensorInitial(const Transform &t);

	const Submap& getActiveSubmap() const;
	const SubmapCollection& getSubmaps() const;
	SubmapCollection* getSubmapsPtr();
	PointCloud getAssembledMapPointCloud() const;
	MapperParameters *getParametersPtr();
	Transform getMapToOdom(const Time &timestamp) const;
	Transform getMapToRangeSensor(const Time &timestamp) const;
	const TransformInterpolationBuffer& getMapToRangeSensorBuffer() const;
	const PointCloud& getPreprocessedScan() const;
	const ScanToMapRegistration &getScanToMapRegistration() const;

	void loopClosureUpdate(const Transform &loopClosureCorrection);
	bool hasProcessedMeasurements() const;
	bool addRangeMeasurement(const PointCloud &cloud, const Time &timestamp);
	
private:
	void update(const MapperParameters &p);
	void checkTransformChainingAndPrintResult(bool isCheckTransformChainingAndPrintResult) const;

	Time lastMeasurementTimestamp_;
	Transform mapToRangeSensor_ = Transform::Identity();
	Transform mapToRangeSensorPrev_ = Transform::Identity();
	Transform mapToRangeSensorLastScanInsertion_ = Transform::Identity();

	MapperParameters params_;
	std::mutex mapManipulationMutex_;
	std::shared_ptr<SubmapCollection> submaps_;
	const TransformInterpolationBuffer &odomToRangeSensorBuffer_;
	TransformInterpolationBuffer mapToRangeSensorBuffer_;
	open3d::geometry::PointCloud preProcessedScan_;
	bool isNewInitialValueSet_ = false;
	bool isIgnoreOdometryPrediction_ = false;
	std::shared_ptr<ScanToMapRegistration> scan2MapReg_;

};

} /* namespace o3d_slam */
