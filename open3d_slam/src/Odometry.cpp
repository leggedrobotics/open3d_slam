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
#include "open3d_slam/output.hpp"

#include <iostream>

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

	const o3d_slam::Timer timer;
	auto croppedCloud = cropper_->crop(cloud);
	// TODO(lukaszpi): create initialization settings for map initialization
	double voxel_size = params_.scanProcessing_.voxelSize_;
	double ratio = params_.scanProcessing_.downSamplingRatio_;
	if(resetIcpTransform_)
	{
		voxel_size = 0.05;
		ratio = 1.0;
	}
	o3d_slam::voxelize(voxel_size, croppedCloud.get());
	auto downSampledCloud = croppedCloud->RandomDownSample(ratio);
	
	if (params_.scanMatcher_.icpObjective_ == o3d_slam::IcpObjective::PointToPlane) {
		o3d_slam::estimateNormals(params_.scanMatcher_.kNNnormalEstimation_, downSampledCloud.get());
		downSampledCloud->NormalizeNormals();
	}

	open3d::pipelines::registration::ICPConvergenceCriteria temp_criteria = icpConvergenceCriteria_;
	double max_correspondance_distance = params_.scanMatcher_.maxCorrespondenceDistance_;
	if(resetIcpTransform_)
	{
		temp_criteria.max_iteration_ = 50;
		max_correspondance_distance = 0.2;
	}
	
	auto result = open3d::pipelines::registration::RegistrationICP(
		cloudPrev_, *downSampledCloud, max_correspondance_distance,
		icpTransform_, *icpObjective_, temp_criteria);

	if(resetIcpTransform_)
	{
		std::cout << "\n\nMY Transform: \n" << asString(Transform(result.transformation_)) << "\n" << std::endl;
	}
	//todo magic
	const bool isOdomOkay = result.fitness_ > params_.minAcceptableFitness_ || resetIcpTransform_;
	if (!isOdomOkay) {
		  std::cout << "Odometry failed!!!!! \n";
			std::cout << "Size of the odom buffer: " << odomToRangeSensorBuffer_.size() << std::endl;
			std::cout << "Scan matching time elapsed: " << timer.elapsedMsec() << " msec \n";
			std::cout << "Fitness: " << result.fitness_ << "\n";
			std::cout << "RMSE: " << result.inlier_rmse_ << "\n";
			std::cout << "Transform: \n" << asString(Transform(result.transformation_)) << "\n";
			std::cout << "target size: " << cloud.points_.size() << std::endl;
			std::cout << "reference size: " << cloudPrev_.points_.size() << std::endl;
			std::cout << "\n \n";
		if (!downSampledCloud->IsEmpty()){
			cloudPrev_ = std::move(*downSampledCloud);
		}
		return isOdomOkay;
	}

	if (resetIcpTransform_)
	{
		icpTransform_ = Eigen::Matrix4d::Identity();
		resetIcpTransform_ = false;
	}
	odomToRangeSensorCumulative_.matrix() *= result.transformation_.inverse();
	cloudPrev_ = std::move(*downSampledCloud);
	odomToRangeSensorBuffer_.push(timestamp, odomToRangeSensorCumulative_);
	lastMeasurementTimestamp_ = timestamp;
	return isOdomOkay;
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

bool  LidarOdometry::hasProcessedMeasurements() const{
	return !odomToRangeSensorBuffer_.empty();
}

void LidarOdometry::setParameters(const OdometryParameters &p) {
	params_ = p;
	icpConvergenceCriteria_.max_iteration_ = p.scanMatcher_.maxNumIter_;
	icpObjective_ = icpObjectiveFactory(params_.scanMatcher_.icpObjective_);
//	cropper_ = std::make_shared<MaxRadiusCroppingVolume>(params_.scanProcessing_.croppingRadius_);
//	cropper_ = std::make_shared<CylinderCroppingVolume>(params_.scanProcessing_.croppingRadius_,-3.0,3.0);
	const auto &par = params_.scanProcessing_.cropper_;
	cropper_ = croppingVolumeFactory(par);
}


void LidarOdometry::setInitialTransform(const Eigen::Matrix4d &initialTransform) {
	icpTransform_ = initialTransform;
	resetIcpTransform_ = true;
}

} // namespace o3d_slam
