/*
 * Odometry.cpp
 *
 *  Created on: Oct 15, 2021
 *      Author: jelavice
 */

#include "m545_volumetric_mapping/Odometry.hpp"
#include "m545_volumetric_mapping/frames.hpp"
#include "m545_volumetric_mapping/helpers.hpp"
#include "m545_volumetric_mapping/time.hpp"
#include <open3d/geometry/BoundingVolume.h>

namespace m545_mapping {

LidarOdometry::LidarOdometry() {
	icpObjective_ = icpObjectiveFactory(IcpObjective::PointToPlane);
	cropper_ = std::make_shared<CroppingVolume>();
}

bool LidarOdometry::addRangeScan(const open3d::geometry::PointCloud &cloud, const ros::Time &timestamp) {
	if (cloudPrev_.IsEmpty()) {
		cloudPrev_ = cloud;
		return true;
	}
//	const m545_mapping::Timer timer("scan_to_scan_odometry");
	auto croppedCloud = cropper_->crop(cloud);
	m545_mapping::voxelize(params_.scanProcessing_.voxelSize_, croppedCloud.get());
	auto downSampledCloud = croppedCloud->RandomDownSample(params_.scanProcessing_.downSamplingRatio_);

	if (params_.scanMatcher_.icpObjective_ == m545_mapping::IcpObjective::PointToPlane) {
		m545_mapping::estimateNormals(params_.scanMatcher_.kNNnormalEstimation_, downSampledCloud.get());
		downSampledCloud->NormalizeNormals();
	}
	auto result = open3d::pipelines::registration::RegistrationICP(cloudPrev_, *downSampledCloud,
			params_.scanMatcher_.maxCorrespondenceDistance_, Eigen::Matrix4d::Identity(), *icpObjective_, icpConvergenceCriteria_);

	//	std::cout << "Scan to scan matching finished \n";
	//	std::cout << "Time elapsed: " << timer.elapsedMsec() << " msec \n";
	//	std::cout << "Fitness: " << result.fitness_ << "\n";
	//	std::cout << "RMSE: " << result.inlier_rmse_ << "\n";
	//	std::cout << "Transform: " << result.transformation_ << "\n";
	//	std::cout << "target size: " << cloud.points_.size() << std::endl;
	//	std::cout << "reference size: " << cloudPrev.points_.size() << std::endl;
	//	std::cout << "\n \n";
	if (result.fitness_ <= 1e-2) {
		return false;
	}
	odomToRangeSensor_.matrix() *= result.transformation_.inverse();
	cloudPrev_ = std::move(*downSampledCloud);
	return true;
}
const Eigen::Isometry3d& LidarOdometry::getOdomToRangeSensor() const {
	return odomToRangeSensor_;
}

const open3d::geometry::PointCloud& LidarOdometry::getPreProcessedCloud() const {
	return cloudPrev_;
}

void LidarOdometry::setParameters(const OdometryParameters &p) {
	params_ = p;
	icpConvergenceCriteria_.max_iteration_ = p.scanMatcher_.maxNumIter_;
	icpObjective_ = icpObjectiveFactory(params_.scanMatcher_.icpObjective_);
	cropper_ = std::make_shared<MaxRadiusCroppingVolume>(params_.scanProcessing_.croppingRadius_);
}

} // namespace m545_mapping
