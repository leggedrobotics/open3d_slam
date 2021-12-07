/*
 * icp_benchmark_node.cpp
 *
 *  Created on: Dec 7, 2021
 *      Author: jelavice
 */

/*
 * mapping_node.cpp
 *
 *  Created on: Sep 1, 2021
 *      Author: jelavice
 */
#include <Eigen/Dense>
#include <open3d/Open3D.h>
#include <ros/ros.h>
#include "open3d_conversions/open3d_conversions.h"
#include <sensor_msgs/PointCloud2.h>
#include "m545_volumetric_mapping/helpers_ros.hpp"
#include "m545_volumetric_mapping/time.hpp"
#include <random>

namespace {

std::mt19937 rndGen;
const std::string frameId = "map";
const open3d::core::Dtype scalarType = open3d::core::Dtype::Float32;
const open3d::core::Device::DeviceType deviceType = open3d::core::Device::DeviceType::CUDA;

enum class IcpExecutionDevice {
	CPU, GPU
};

namespace registration = open3d::t::pipelines::registration;
using PointCloud = open3d::geometry::PointCloud;
using tPointCloud = open3d::t::geometry::PointCloud;
open3d::pipelines::registration::ICPConvergenceCriteria icpConvergenceCriteria;

open3d::geometry::PointCloud load(const std::string &filename) {
	auto retVal = open3d::io::CreatePointCloudFromFile(filename);
	return std::move(*retVal);
}

PointCloud fromTensor(const tPointCloud &in) {
	return in.ToLegacyPointCloud();
}

tPointCloud toTensor(const PointCloud &in) {
	open3d::core::Device device(deviceType, 0);
	return tPointCloud::FromLegacyPointCloud(in, scalarType,
			device);
}

registration::RegistrationResult toTensor(
		const open3d::pipelines::registration::RegistrationResult &in) {
	registration::RegistrationResult out;
//	out.correspondence_set_ = in.correspondence_set_;

	return out;

}

open3d::core::Tensor toTensor(const Eigen::Matrix4d &in) {

	open3d::core::Device device(deviceType, 0);
	open3d::core::Tensor out = open3d::core::Tensor::Eye(4, scalarType, device);
	for (size_t row = 0; row < 4; ++row)
		for (size_t col = 0; col < 4; ++col)
			out[row][col] = in(row, col);
	return out;
}

open3d::t::pipelines::registration::ICPConvergenceCriteria toTensor(
		const open3d::pipelines::registration::ICPConvergenceCriteria &in) {

	open3d::t::pipelines::registration::ICPConvergenceCriteria out;
	out.max_iteration_ = in.max_iteration_;
	out.relative_fitness_ = in.relative_fitness_;
	out.relative_rmse_ = in.relative_rmse_;
	return out;
}

std::unique_ptr<registration::TransformationEstimation> toTensor(
		const open3d::pipelines::registration::TransformationEstimation &in) {
	using TransformationEstimationType = open3d::pipelines::registration::TransformationEstimationType;
	switch (in.GetTransformationEstimationType()) {
	case TransformationEstimationType::PointToPoint: {
		return std::make_unique<
				registration::TransformationEstimationPointToPoint>();
	}
	case TransformationEstimationType::PointToPlane: {
		return std::make_unique<
				registration::TransformationEstimationPointToPlane>();
	}
	default:
		throw std::runtime_error("Unsupported transformation estimation type");
	}

}

open3d::pipelines::registration::RegistrationResult icpRegistration(
		const PointCloud &source, const PointCloud &target,
		double maxCorrespondenceDistance, const Eigen::Matrix4d &init,
		const open3d::pipelines::registration::TransformationEstimation &transformationEstimation,
		const open3d::pipelines::registration::ICPConvergenceCriteria &icpConvergenceCriteria,
		IcpExecutionDevice device) {

	switch (device) {
	case IcpExecutionDevice::CPU: {
		return open3d::pipelines::registration::RegistrationICP(source, target,
				maxCorrespondenceDistance, init, transformationEstimation,
				icpConvergenceCriteria);
	}
	case IcpExecutionDevice::GPU: {
		const auto sourceT = toTensor(source);
		const auto targetT = toTensor(target);
		const auto icpConvergenceCriteriaT = toTensor(icpConvergenceCriteria);
		const auto transformationEstimationT = toTensor(
				transformationEstimation);
		const auto initT = toTensor(init);

		auto res = registration::RegistrationICP(sourceT, targetT,
				maxCorrespondenceDistance, initT, *transformationEstimationT,
				icpConvergenceCriteriaT);

		open3d::pipelines::registration::RegistrationResult out;
		out.fitness_ = res.fitness_;
		out.inlier_rmse_ = res.inlier_rmse_;


		return out;

	}
	default:
		throw std::runtime_error("Unknonwn execution device for the icp");
	}

}

Eigen::Matrix4d getPerturbation() {
	const double M_PI_8 = M_PI_4 * 0.5;
	std::uniform_real_distribution<double> rollDist(-M_PI_8, M_PI_8);
	std::uniform_real_distribution<double> pitchDist(-M_PI_8, M_PI_8);
	const double roll = rollDist(rndGen);
	const double pitch = pitchDist(rndGen);
	const auto xyz = Eigen::Vector3d::Random();
	const auto rot = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
			* Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY());
	Eigen::Isometry3d T(rot);
	T.translation() = xyz;
//	return Eigen::Matrix4d::Identity();
	return T.matrix();
}

} // namespace
int main(int argc, char **argv) {
	ros::init(argc, argv, "icp_benchmark_node");
	rndGen.seed(23452340);

	ros::NodeHandle nh("~");
	ros::Publisher targetPub = nh.advertise<sensor_msgs::PointCloud2>("target",
			1, true);
	ros::Publisher sourcePub = nh.advertise<sensor_msgs::PointCloud2>("source",
			1, true);

	const std::string cloudFilename = nh.param<std::string>("cloud_filename",
			"");

	auto target = load(cloudFilename);
	const auto targetCenter = target.GetCenter();
	target.Translate(-targetCenter);
//	auto voxelized = target.VoxelDownSample(0.2);
//	target = *voxelized;
	target.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.4, 5));
	icpConvergenceCriteria.max_iteration_ = 50;

	m545_mapping::Timer tCPU, tGPU;
	const size_t numTests = 100;
	for (size_t i = 0; i < numTests; ++i) {
		const auto pertrubation = getPerturbation();
		auto source = target; // copy
		source.Transform(pertrubation);

		tCPU.startStopwatch();
		auto resCPU =
				icpRegistration(source, target, 2.0,
						Eigen::Matrix4d::Identity(),
						open3d::pipelines::registration::TransformationEstimationPointToPlane(),
						icpConvergenceCriteria, IcpExecutionDevice::CPU);
		tCPU.addMeasurementMsec(tCPU.elapsedMsecSinceStopwatchStart());

		tGPU.startStopwatch();
		auto resGPU =
				icpRegistration(source, target, 2.0,
						Eigen::Matrix4d::Identity(),
						open3d::pipelines::registration::TransformationEstimationPointToPlane(),
						icpConvergenceCriteria, IcpExecutionDevice::GPU);
		tGPU.addMeasurementMsec(tGPU.elapsedMsecSinceStopwatchStart());

//		const auto timestamp = ros::Time::now();
//		m545_mapping::publishCloud(target, frameId, timestamp, targetPub);
//		m545_mapping::publishCloud(source, frameId, timestamp, sourcePub);
//		ros::spinOnce();
//		sleep(2);

	}
	std::cout << "Cloud size: " << target.points_.size() << std::endl;
	std::cout << "Avg cpu icp time: " << tCPU.getAvgMeasurementMsec()
			<< " msec \n";
	std::cout << "Avg gpu icp time: " << tGPU.getAvgMeasurementMsec()
			<< " msec \n";


	ros::spin();

	return 0;
}

