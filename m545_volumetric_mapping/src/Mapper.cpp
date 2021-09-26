/*
 * Mapper.cpp
 *
 *  Created on: Sep 26, 2021
 *      Author: jelavice
 */

#include "m545_volumetric_mapping/Mapper.hpp"
#include <open3d/Open3D.h>
#include <open3d/pipelines/registration/Registration.h>
#include "m545_volumetric_mapping/frames.hpp"
#include "m545_volumetric_mapping/helpers.hpp"
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>

namespace m545_mapping {

namespace {
	namespace registration = open3d::pipelines::registration;
	std::shared_ptr<registration::TransformationEstimation> icpObjective;
} // namespace

Mapper::Mapper() :
		tfListener_(tfBuffer_) {
	icpObjective = icpObjectiveFactory(params_.icpObjective_);
}

void Mapper::setParameters(const IcpParameters &p){
	params_ = p;
	icpObjective = icpObjectiveFactory(params_.icpObjective_);
}

void Mapper::addRangeMeasurement(const Mapper::PointCloud &cloud,const ros::Time &timestamp) {
	lastMeasurementTimestamp_ = timestamp;

	//todo figure out how to estimate nomals
	if (map_.points_.empty()){
		map_ += cloud;
		return;
	}

	const auto odomToRangeSensor = lookupTransform(frames::rangeSensorFrame, frames::odomFrame, ros::Time(0));
	const auto odometryMotion = odomToRangeSensorPrev_.inverse() * odomToRangeSensor;
	const bool isMovedTooLittle = odometryMotion.translation().norm() < 0.3;
	if (isMovedTooLittle){
		odomToRangeSensorPrev_ = odomToRangeSensor;
		return;
	}

	// todo properly fix frames
	auto criteria = open3d::pipelines::registration::ICPConvergenceCriteria();
	criteria.max_iteration_ = params_.maxNumIter_;
	open3d::geometry::AxisAlignedBoundingBox bbox;
	bbox.min_bound_ = params_.cropBoxLowBound_;
	bbox.max_bound_ = params_.cropBoxHighBound_;
	auto croppedCloud = cloud.Crop(bbox);
	auto downSampledCloud = croppedCloud->RandomDownSample(params_.downSamplingRatio_);
	if (params_.icpObjective_ == m545_mapping::IcpObjective::PointToPlane) {
		estimateNormals(params_.kNNnormalEstimation_, &map_);
		map_.NormalizeNormals();
	}
	auto result = open3d::pipelines::registration::RegistrationICP(*downSampledCloud, map_, params_.maxCorrespondenceDistance_,
			mapToRangeSensor_.matrix(), *icpObjective, criteria);

	odomToRangeSensorPrev_ = odomToRangeSensor;
}
const Mapper::PointCloud& Mapper::getMap() const {
	return map_;
}


Eigen::Isometry3d Mapper::lookupTransform(const std::string& target_frame, const std::string& source_frame,
	    const ros::Time& time) const {
    geometry_msgs::TransformStamped transformStamped;
    try{
      transformStamped = tfBuffer_.lookupTransform(target_frame, source_frame,
                               time);
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("caught exception while looking up the tf: %s",ex.what());
      return Eigen::Isometry3d::Identity();
    }

    return tf2::transformToEigen(transformStamped);
}

} /* namespace m545_mapping */
