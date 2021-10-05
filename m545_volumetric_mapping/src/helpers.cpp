/*
 * helpers.cpp
 *
 *  Created on: Sep 26, 2021
 *      Author: jelavice
 */

#include "m545_volumetric_mapping/helpers.hpp"
#include "m545_volumetric_mapping/output.hpp"

#include <open3d/Open3D.h>
#include <open3d/pipelines/registration/Registration.h>

namespace m545_mapping {

namespace {
namespace registration = open3d::pipelines::registration;
}//namespace

void cropPointcloud(const open3d::geometry::AxisAlignedBoundingBox &bbox, open3d::geometry::PointCloud *pcl){
	auto croppedCloud = pcl->Crop(bbox);
	*pcl = *croppedCloud;
}


std::string asString (const Eigen::Isometry3d &T){
    const double kRadToDeg = 180.0 / M_PI;
    const auto &t = T.translation();
    const auto &q = Eigen::Quaterniond(T.rotation());
    const std::string trans  =  string_format("t:[%f, %f, %f]", t.x(), t.y(), t.z());
    const std::string rot = string_format("q:[%f, %f, %f, %f]",q.x(), q.y(), q.z(), q.w());
    const auto rpy = toRPY(q) * kRadToDeg;
    const std::string rpyString = string_format("rpy (deg):[%f, %f, %f]",rpy.x(), rpy.y(),rpy.z());
    return trans + " ; " + rot + " ; " + rpyString;


}

Eigen::Quaterniond fromRPY(const double roll, const double pitch, const double yaw)
{

  const Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());
  return yaw_angle * pitch_angle * roll_angle;
}

Eigen::Vector3d toRPY(const Eigen::Quaterniond &_q)
{
  Eigen::Quaterniond q(_q);
  q.normalize();
  const double r = getRollFromQuat(q.w(), q.x(), q.y(), q.z());
  const double p = getPitchFromQuat(q.w(), q.x(), q.y(), q.z());
  const double y = getYawFromQuat(q.w(), q.x(), q.y(), q.z());
  return Eigen::Vector3d(r,p,y);
}

Eigen::Quaterniond fromRPY(const Eigen::Vector3d &rpy){
	return fromRPY(rpy.x(), rpy.y(), rpy.z());
}

void estimateNormals(int numNearestNeighbours, open3d::geometry::PointCloud *pcl) {
	open3d::geometry::KDTreeSearchParamKNN param(numNearestNeighbours);
	pcl->EstimateNormals(param);
}

std::shared_ptr<registration::TransformationEstimation> icpObjectiveFactory(
		const m545_mapping::IcpObjective &obj) {

	switch (obj) {
	case m545_mapping::IcpObjective::PointToPoint: {
		auto obj = std::make_shared<registration::TransformationEstimationPointToPoint>(false);
		return obj;
	}

	case m545_mapping::IcpObjective::PointToPlane: {
		auto obj = std::make_shared<registration::TransformationEstimationPointToPlane>();
		return obj;
	}

	default:
		throw std::runtime_error("Unknown icp objective");
	}

}

Timer::Timer(bool isPrintInDestructor){
	startTime_ = std::chrono::steady_clock::now();
	isPrintInDestructor_ = isPrintInDestructor;
}
Timer::~Timer(){
	if(isPrintInDestructor_){
		std::cout<< "Timer: Elapsed time: " << elapsedMsec() << "msec \n";
	}
}

double Timer::elapsedMsec() const{
	const auto endTime = std::chrono::steady_clock::now();
	return std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime_).count() / 1e3;
}
double Timer::elapsedSec() const{
	const auto endTime = std::chrono::steady_clock::now();
	return std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime_).count() / 1e3;
}

} /* namespace m545_mapping */


