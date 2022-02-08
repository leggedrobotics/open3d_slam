/*
 * Odometry.cpp
 *
 *  Created on: Oct 15, 2021
 *      Author: jelavice
 */

#include "open3d_slam/Odometry.hpp"
#include "open3d_slam/frames.hpp"
#include "open3d_slam/helpers.hpp"
#include "open3d_slam/time.hpp"
#include <open3d/geometry/BoundingVolume.h>

namespace o3d_slam {

LidarOdometry::LidarOdometry() {
	icpObjective_ = icpObjectiveFactory(IcpObjective::PointToPlane);
	cropper_ = std::make_shared<CroppingVolume>();
}

bool LidarOdometry::addRangeScan(const open3d::geometry::PointCloud &cloud, const Time &timestamp) {
	if (cloudPrev_.IsEmpty()) {
		cloudPrev_ = cloud;
		odomToRangeSensorBuffer_.push(timestamp, odomToRangeSensorCumulative_);
		lastMeasurementTimestamp_ = timestamp;
		return true;
	}

	if (timestamp < lastMeasurementTimestamp_) {
			std::cerr << "\n\n !!!!! LIDAR ODOMETRY WARNING: Measurements came out of order!!!! \n\n";
			return false;
	}

//	const o3d_slam::Timer timer("scan_to_scan_odometry");
	auto croppedCloud = cropper_->crop(cloud);
	o3d_slam::voxelize(params_.scanProcessing_.voxelSize_, croppedCloud.get());
	auto downSampledCloud = croppedCloud->RandomDownSample(params_.scanProcessing_.downSamplingRatio_);

	if (params_.scanMatcher_.icpObjective_ == o3d_slam::IcpObjective::PointToPlane) {
		o3d_slam::estimateNormals(params_.scanMatcher_.kNNnormalEstimation_, downSampledCloud.get());
		downSampledCloud->NormalizeNormals();
	}
	auto result = open3d::pipelines::registration::RegistrationICP(cloudPrev_, *downSampledCloud,
			params_.scanMatcher_.maxCorrespondenceDistance_, Eigen::Matrix4d::Identity(), *icpObjective_,
			icpConvergenceCriteria_);

	//	std::cout << "Scan to scan matching finished \n";
	//	std::cout << "Time elapsed: " << timer.elapsedMsec() << " msec \n";
	//	std::cout << "Fitness: " << result.fitness_ << "\n";
	//	std::cout << "RMSE: " << result.inlier_rmse_ << "\n";
	//	std::cout << "Transform: " << result.transformation_ << "\n";
	//	std::cout << "target size: " << cloud.points_.size() << std::endl;
	//	std::cout << "reference size: " << cloudPrev.points_.size() << std::endl;
	//	std::cout << "\n \n";
	if (result.fitness_ <= 0.1) {
		return false;
	}
	odomToRangeSensorCumulative_.matrix() *= result.transformation_.inverse();
	cloudPrev_ = std::move(*downSampledCloud);
	odomToRangeSensorBuffer_.push(timestamp, odomToRangeSensorCumulative_);
	lastMeasurementTimestamp_ = timestamp;
	return true;
}
const Transform LidarOdometry::getOdomToRangeSensor(const Time &t) const {
	return getTransform(t, odomToRangeSensorBuffer_);
}

const open3d::geometry::PointCloud& LidarOdometry::getPreProcessedCloud() const {
	return cloudPrev_;
}

const TransformInterpolationBuffer& LidarOdometry::getBuffer() const {
	return odomToRangeSensorBuffer_;
}

void LidarOdometry::setParameters(const OdometryParameters &p) {
	params_ = p;
	icpConvergenceCriteria_.max_iteration_ = p.scanMatcher_.maxNumIter_;
	icpObjective_ = icpObjectiveFactory(params_.scanMatcher_.icpObjective_);
//	cropper_ = std::make_shared<MaxRadiusCroppingVolume>(params_.scanProcessing_.croppingRadius_);
//	cropper_ = std::make_shared<CylinderCroppingVolume>(params_.scanProcessing_.croppingRadius_,-3.0,3.0);
	const auto &par = params_.scanProcessing_.cropper_;
	cropper_ = croppingVolumeFactory(par.cropperName_, par.croppingRadius_, par.croppingMinZ_, par.croppingMaxZ_);
}

} // namespace o3d_slam