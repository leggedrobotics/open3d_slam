/*
 * MotionCompensation.cpp
 *
 *  Created on: Apr 26, 2022
 *      Author: jelavice
 */

#include "open3d_slam/MotionCompensation.hpp"

#include <open3d/geometry/PointCloud.h>
#include <Eigen/Dense>
#include <cmath>
#include <memory>
#include <iostream>
#include "open3d_slam/Parameters.hpp"
#include "open3d_slam/time.hpp"
#include "open3d_slam/math.hpp"
#include "open3d_slam/output.hpp"
#include "open3d_slam/assert.hpp"
#include "open3d_slam/TransformInterpolationBuffer.hpp"

namespace o3d_slam {

std::shared_ptr<PointCloud> MotionCompensation::undistortInputPointCloud(
		const PointCloud &input, const Time &timestamp) {
	std::shared_ptr<PointCloud> ret = std::make_shared<PointCloud>();
	*ret = input;
	return ret;
}

ConstantVelocityMotionCompensation::ConstantVelocityMotionCompensation(
		const TransformInterpolationBuffer &buffer) :
		buffer_(buffer){
}

void ConstantVelocityMotionCompensation::estimateLinearAndAngularVelocity(
		const Time &timestamp, Eigen::Vector3d *linearVelocity,
		Eigen::Vector3d *angularVelocity) const {

	const int offset = params_.numPosesVelocityEstimation_;
	if (buffer_.size() <= offset) {
		linearVelocity->setZero();
		angularVelocity->setZero();
		return;
	}

	if (buffer_.latest_time() < timestamp) {

		const auto finish = buffer_.latest_measurement();
		const auto start = buffer_.latest_measurement(offset);

		const Transform dT = start.transform_.inverse() * finish.transform_;
		const double dt = toSeconds(finish.time_ - start.time_);
		assert_gt(dt, 0.0, "dt should be > 0!!!!");
//			std::cout << "dt " << dt << std::endl;
		const Eigen::Vector3d linearVelocitySensor = dT.translation()
				/ (dt + 1e-6);
		const Eigen::Vector3d angularVelocitySensor = toRPY(
				Eigen::Quaterniond(dT.rotation()).normalized()) / (dt + 1e-6);
		*linearVelocity = linearVelocitySensor;
		*angularVelocity = angularVelocitySensor;
	} else {
		// todo handle this case!!!!!
		std::cout << "Warning buffer has this already!!!! \n";
	}

}

void ConstantVelocityMotionCompensation::setParameters(const ConstantVelocityMotionCompensationParameters &p){
	params_ = p;
	assert_gt<double>(params_.scanDuration_,0.0, "lidar scanDuration_: ");
}


std::shared_ptr<PointCloud> ConstantVelocityMotionCompensation::undistortInputPointCloud(
		const PointCloud &input, const Time &timestamp) {
	auto output = std::make_shared<PointCloud>(input);
	Eigen::Vector3d linearVelocity(0.0,0.0,0.0), angularVelocityRpy(0.0,0.0,0.0);
	estimateLinearAndAngularVelocity(timestamp, &linearVelocity,
			&angularVelocityRpy);
//	std::cout << "lin vel: " << linearVelocity.transpose() << std::endl;
//	std::cout << "ang vel: " << angularVelocityRpy.transpose() * 180.0 / M_PI
//			<< "\n";

//		std::cout << "dt (-0.0002,1): " << computePhase(-0.0002,1) << "\n";
//		std::cout << "dt (0.0002,1): " << computePhase(0.0002,1) << "\n";
//		std::cout << "dt (0,1): " << computePhase(0.0,1.0) << "\n";
//		std::cout << "dt (1,0): " << computePhase(1,0) << "\n";
//		std::cout << "dt (0,-1): " << computePhase(0,-1) << "\n";
//		std::cout << "dt (-1,0): " << computePhase(-1,0) << "\n\n";
//	double minPhase = 2.0;
//	double maxPhase = -1.0;
//	Transform minT, maxT;
//	Eigen::Vector3d pMin, pMax;
	for (int i = 0; i < input.points_.size(); ++i) {
		const Eigen::Vector3d p = output->points_.at(i);
		const double phase = computePhase(p.x(), p.y());
		const Eigen::Vector3d xyz = phase * params_.scanDuration_ * linearVelocity;
		const Eigen::Vector3d rpy = phase * params_.scanDuration_ * angularVelocityRpy;
		const Transform motion = makeTransform(xyz, fromRPY(rpy).normalized());
		const Transform point = makeTransform(p,
				Eigen::Quaterniond::Identity());
//		if (phase < minPhase) {
//			minT = motion;
//			minPhase = phase;
//			pMin = p;
//		}
//		if (phase > maxPhase) {
//			maxT = motion;
//			maxPhase = phase;
//			pMax = p;
//		}
		output->points_.at(i) = motion * p;
	}

//	if (std::fabs(angularVelocityRpy.z()) > 160 * M_PI / 180.0) {
//		const std::string folder =
//				"/home/jelavice/catkin_workspaces/open3d_ws/src/open3d_slam/open3d_slam_ros/data";
//		saveToFile(folder + "/input.pcd", input);
//		saveToFile(folder + "/output.pcd", *output);
//		std::cout << "saved \n";
//	}

//	std::cout << "min motion: \n" << asString(minT) << "\n";
//	std::cout << "p min: " << pMin.transpose() << ", "
//			<< (minT * pMin).transpose() << std::endl;
//	std::cout << "max motion: \n" << asString(maxT) << "\n";
//	std::cout << "p max: " << pMax.transpose() << ", "
//			<< (maxT * pMax).transpose() << std::endl;
//	std::cout << "\n\n";
	return output;

}

double ConstantVelocityMotionCompensation::computePhase(double x, double y) {
	//this is now robosense specific
	const double angle = std::atan2(y, x);
	const double angleWrapped = angle < 0.0 ? (angle + 2.0 * M_PI) : angle;
	if (angleWrapped == 0.0) {
		return 0.0;
	}
	double phase = -5.0;

	if (params_.isSpinningClockwise_){
		phase = 1.0 - angleWrapped / (2.0 * M_PI);
	} else {
		phase = angleWrapped / (2.0 * M_PI);
	}

	assert_le(phase, 1.0, "phase should be   <= 1.0");
	assert_ge(phase, 0.0, "phase should be   >= 0.0");
//		std::cout << "point: " << p.x() << ", " << p.y() << " has phase: " <<  phase << std::endl;
	return phase;

}

} // namespace o3d_slam

