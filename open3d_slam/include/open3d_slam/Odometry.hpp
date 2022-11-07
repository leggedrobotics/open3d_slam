/*
 * Odometry.hpp
 *
 *  Created on: Sep 2, 2021
 *      Author: jelavice
 */

#pragma once

#include <open3d/geometry/PointCloud.h>
#include <Eigen/Dense>
#include "open3d_slam/Parameters.hpp"
#include "open3d_slam/croppers.hpp"
#include "open3d_slam/TransformInterpolationBuffer.hpp"

namespace o3d_slam {

class CloudRegistration;

class LidarOdometry {

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	LidarOdometry();
	~LidarOdometry() = default;

	bool addRangeScan(const open3d::geometry::PointCloud &cloud, const Time &timestamp);
	const Transform getOdomToRangeSensor(const Time &t) const;
	const open3d::geometry::PointCloud &getPreProcessedCloud() const;
	void setParameters (const OdometryParameters &p);
	const TransformInterpolationBuffer &getBuffer() const;
	bool hasProcessedMeasurements() const;
	void setInitialTransform(const Eigen::Matrix4d &initialTransform);

private:

	PointCloudPtr preprocess(const PointCloud &in) const;

	TransformInterpolationBuffer odomToRangeSensorBuffer_;
	open3d::geometry::PointCloud cloudPrev_;
	Transform odomToRangeSensorCumulative_ = Transform::Identity();
	OdometryParameters params_;
	std::shared_ptr<CroppingVolume> cropper_;
	Time lastMeasurementTimestamp_;
	Eigen::Matrix4d initialTransform_ = Eigen::Matrix4d::Identity();
	bool isInitialTransformSet_ = false;
	std::shared_ptr<CloudRegistration> cloudRegistration_;
};

} // namespace o3d_slam
