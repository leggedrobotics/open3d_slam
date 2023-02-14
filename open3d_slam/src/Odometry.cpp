

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
#include "open3d_slam/CloudRegistration.hpp"

#include <iostream>

namespace o3d_slam {

LidarOdometry::LidarOdometry() {
	cropper_ = std::make_shared<CroppingVolume>();
	cloudRegistration_ = cloudRegistrationFactory(params_.scanMatcher_);
}

PointCloudPtr LidarOdometry::preprocess(const PointCloud &in) const{
	auto croppedCloud = cropper_->crop(in);
	o3d_slam::voxelize(params_.scanProcessing_.voxelSize_, croppedCloud.get());
	cloudRegistration_->estimateNormalsOrCovariancesIfNeeded(croppedCloud.get());
	return croppedCloud->RandomDownSample(params_.scanProcessing_.downSamplingRatio_);
}

bool LidarOdometry::addRangeScan(const open3d::geometry::PointCloud &cloud, const Time &timestamp) {
	if (cloudPrev_.IsEmpty()) {
		auto preProcessed = preprocess(cloud);
		cloudPrev_ = *preProcessed;
		odomToRangeSensorBuffer_.push(timestamp, odomToRangeSensorCumulative_);
		lastMeasurementTimestamp_ = timestamp;
		return true;
	}

	if (timestamp < lastMeasurementTimestamp_) {
			std::cerr << "\n\n !!!!! LIDAR ODOMETRY WARNING: Measurements came out of order!!!! \n\n";
			return false;
	}

	const o3d_slam::Timer timer;
	auto preProcessed = preprocess(cloud);
	const auto result = cloudRegistration_->registerClouds(cloudPrev_, *preProcessed, Transform::Identity());
	auto cov = estimateCovariance(*preProcessed, cloudPrev_, result.transformation_);

	//todo magic
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
		if (!preProcessed->IsEmpty()){
			cloudPrev_ = std::move(*preProcessed);
		}
		return isOdomOkay;
	}

	if (isInitialTransformSet_){
		odomToRangeSensorCumulative_.matrix() = initialTransform_;
		isInitialTransformSet_ = false;
	} else {
		odomToRangeSensorCumulative_.matrix() *= result.transformation_.inverse();
	}

	cloudPrev_ = std::move(*preProcessed);
	odomToRangeSensorBuffer_.push(timestamp, odomToRangeSensorCumulative_);
	covariances_.insert(std::pair<Time, Matrix6d>(timestamp, cov));
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

const std::map<Time, Matrix6d>& LidarOdometry::getCovarianceBuffer() const {
	return covariances_;
}

bool LidarOdometry::hasProcessedMeasurements() const {
	return !odomToRangeSensorBuffer_.empty();
}

void LidarOdometry::setParameters(const OdometryParameters &p) {
	params_ = p;
	cropper_ = croppingVolumeFactory(params_.scanProcessing_.cropper_);
	cloudRegistration_ = cloudRegistrationFactory(params_.scanMatcher_);
}


void LidarOdometry::setInitialTransform(const Eigen::Matrix4d &initialTransform) {
	//todo decide what to do
	// if I uncomment stuff below the odom jumps but starts from the pose you specified
	// if I leave it like this it is always continuous, but starts always from the
	// origin
	initialTransform_ = initialTransform;
	odomToRangeSensorCumulative_ = Transform(initialTransform);
	isInitialTransformSet_ = true;
}
Matrix6d LidarOdometry::estimateCovariance(const PointCloud& ptIn, const PointCloud& registered, const Eigen::Matrix4d transformation) {
	const int max_nbr_point = ptIn.points_.size();

	Matrix6d covariance(Matrix6d::Zero());
	Matrix6d J_hessian(Matrix6d::Zero());
	Eigen::MatrixXd d2J_dReadingdX(Eigen::MatrixXd::Zero(6, max_nbr_point));
	Eigen::MatrixXd d2J_dReferencedX(Eigen::MatrixXd::Zero(6, max_nbr_point));

	Eigen::Vector3d reading_point(Eigen::Vector3d::Zero());
	Eigen::Vector3d reference_point(Eigen::Vector3d::Zero());
	Eigen::Vector3d normal;
	Eigen::Vector3d reading_direction(Eigen::Vector3d::Zero());
	Eigen::Vector3d reference_direction(Eigen::Vector3d::Zero());
	Eigen::Vector6d tmp_vector_6(Eigen::Vector6d::Zero());

	Eigen::MatrixXd d2J_dZdX_bias_reading(Eigen::MatrixXd::Zero(6, 1));
	Eigen::MatrixXd d2J_dZdX_bias_reference(Eigen::MatrixXd::Zero(6, 1));

	int valid_points_count = 0;
	for (int i = 0; i < max_nbr_point; ++i) {
		reading_point = ptIn.points_[i];
		reference_point = registered.points_[i];

		normal = registered.normals_[i];
		double reading_range = reading_point.norm();
		reading_direction = reading_point / reading_range;
		double reference_range = reference_point.norm();
		reference_direction = reference_point / reference_range;

		Eigen::Vector3d tmp_vector_3 = transformation.block(0, 0, 3, 3) * reference_point + transformation.block(0, 3, 3, 1);
		double n_alpha = normal(2) * tmp_vector_3(1) - normal(1) * tmp_vector_3(2);
		double n_beta = normal(0) * tmp_vector_3(2) - normal(2) * tmp_vector_3(0);
		double n_gamma = normal(1) * tmp_vector_3(0) - normal(0) * tmp_vector_3(1);

		// update the hessian and d2J/dzdx
		tmp_vector_6 << reading_range * n_alpha, reading_range * n_beta, reading_range * n_gamma, normal(0), normal(1), normal(2);
		J_hessian += tmp_vector_6 * tmp_vector_6.transpose();

		double tmp_scalar_read = normal(0) * reading_direction(0) + normal(1) * reading_direction(1) + normal(2) * reading_direction(2);
		tmp_scalar_read = 1;
		d2J_dReadingdX.block(0, valid_points_count, 6, 1) = tmp_vector_6 * tmp_scalar_read;
		d2J_dZdX_bias_reading += tmp_vector_6 * tmp_scalar_read;

		double tmp_scalar_ref = normal(0) * reference_direction(0) + normal(1) * reference_direction(1) + normal(2) * reference_direction(2);
		tmp_scalar_ref = 1;
		d2J_dReferencedX.block(0, valid_points_count, 6, 1) = tmp_vector_6 * tmp_scalar_ref;
		d2J_dZdX_bias_reference += tmp_vector_6 * tmp_scalar_ref;
		valid_points_count++;
	}

	Eigen::MatrixXd d2J_dZdX(Eigen::MatrixXd::Zero(6, 2 * valid_points_count));
	d2J_dZdX.block(0, 0, 6, valid_points_count) = d2J_dReadingdX.block(0, 0, 6, valid_points_count);
	d2J_dZdX.block(0, valid_points_count, 6, valid_points_count) = d2J_dReferencedX.block(0, 0, 6, valid_points_count);

	Matrix6d inv_J_hessian = J_hessian.inverse();

	covariance = d2J_dZdX * d2J_dZdX.transpose();
	Matrix6d censi_cov = inv_J_hessian * covariance * inv_J_hessian;
	Matrix6d bonnabel_cov = Matrix6d::Zero();
	bonnabel_cov += inv_J_hessian * d2J_dZdX_bias_reading * d2J_dZdX_bias_reading.transpose() * inv_J_hessian;
	bonnabel_cov += inv_J_hessian * d2J_dZdX_bias_reference * d2J_dZdX_bias_reference.transpose() * inv_J_hessian;
	bonnabel_cov += censi_cov;
	return bonnabel_cov;
}

} // namespace o3d_slam
