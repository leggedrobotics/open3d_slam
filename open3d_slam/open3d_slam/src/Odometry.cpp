

/*
 * Odometry.cpp
 *
 *  Created on: Oct 15, 2021
 *      Author: jelavice
 */
#include "open3d_slam/Odometry.hpp"
#include "open3d_slam/CloudRegistration.hpp"
#include "open3d_slam/frames.hpp"
#include "open3d_slam/helpers.hpp"
#include "open3d_slam/output.hpp"
#include "open3d_slam/time.hpp"

#include <iostream>

namespace o3d_slam {

LidarOdometry::LidarOdometry() {
  // cropper_ = std::make_shared<CroppingVolume>();
  cloudRegistration_ = cloudRegistrationFactory(params_.scanMatcher_);
}

PointCloudPtr LidarOdometry::preprocess(const PointCloud& in) const {
  auto croppedCloudPtr = cropper_->crop(in);
  o3d_slam::voxelize(params_.scanProcessing_.voxelSize_, croppedCloudPtr);
  cloudRegistration_->estimateNormalsOrCovariancesIfNeeded(croppedCloudPtr.get());
  return croppedCloudPtr->RandomDownSample(params_.scanProcessing_.downSamplingRatio_);
}

bool LidarOdometry::addRangeScan(const open3d::geometry::PointCloud& cloud, const Time& timestamp) {
  // Check
  if (timestamp < lastMeasurementTimestamp_) {
    std::cerr << "\n\n !!!!! LIDAR ODOMETRY WARNING: Measurements came out of order!!!! \n\n";
    return false;
  }
  // Implementation
  std::shared_ptr<PointCloud> preProcessedPtr = preprocess(cloud);
  if (cloudPrev_.IsEmpty()) {  // First iteration
    cloudPrev_ = *preProcessedPtr;
    odomToRangeSensorBuffer_.push(timestamp, odomToRangeSensorCumulative_);
    lastMeasurementTimestamp_ = timestamp;
    return true;
  } else {  // Later iterations
    const o3d_slam::Timer timer;
    const auto result = cloudRegistration_->registerClouds(cloudPrev_, *preProcessedPtr, Transform::Identity());

    // todo magic
    const bool isOdomOkay = result.fitness_ > 0.1;
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
      if (!preProcessedPtr->IsEmpty()) {
        cloudPrev_ = *preProcessedPtr;
      }
      return isOdomOkay;
    }

    if (isInitialTransformSet_) {
      odomToRangeSensorCumulative_.matrix() = initialTransform_;
      isInitialTransformSet_ = false;
    } else {
      odomToRangeSensorCumulative_.matrix() *= result.transformation_.inverse();
    }

    cloudPrev_ = *preProcessedPtr;
    odomToRangeSensorBuffer_.push(timestamp, odomToRangeSensorCumulative_);
    lastMeasurementTimestamp_ = timestamp;
    return isOdomOkay;
  }
}
const Transform LidarOdometry::getOdomToRangeSensor(const Time& t) const {
  return getTransform(t, odomToRangeSensorBuffer_);
}

const open3d::geometry::PointCloud& LidarOdometry::getPreProcessedCloud() const {
  return cloudPrev_;
}

const TransformInterpolationBuffer& LidarOdometry::getBuffer() const {
  return odomToRangeSensorBuffer_;
}

bool LidarOdometry::hasProcessedMeasurements() const {
  return !odomToRangeSensorBuffer_.empty();
}

void LidarOdometry::setParameters(const OdometryParameters& p) {
  params_ = p;
  cropper_ = croppingVolumeFactory(params_.scanProcessing_.cropper_);
  cloudRegistration_ = cloudRegistrationFactory(params_.scanMatcher_);
}

void LidarOdometry::setInitialTransform(const Eigen::Matrix4d& initialTransform) {
  // todo decide what to do
  //  if I uncomment stuff below the odom jumps but starts from the pose you specified
  //  if I leave it like this it is always continuous, but starts always from the
  //  origin
  initialTransform_ = initialTransform;
  odomToRangeSensorCumulative_ = Transform(initialTransform);
  isInitialTransformSet_ = true;
}

}  // namespace o3d_slam
