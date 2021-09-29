/*
 * Mapper.hpp
 *
 *  Created on: Sep 26, 2021
 *      Author: jelavice
 */

#pragma once

#include <Eigen/Geometry>
#include <open3d/geometry/PointCloud.h>
#include <open3d/pipelines/registration/Registration.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Transform.h>
#include "m545_volumetric_mapping/Parameters.hpp"
namespace m545_mapping{


class Mapper {

public:

	using PointCloud = open3d::geometry::PointCloud;

	Mapper();
	~Mapper()=default;

	void addRangeMeasurement(const PointCloud &cloud, const ros::Time &timestamp);
	const PointCloud &getMap() const;
	void setParameters(const MapperParameters &p);
	bool isMatchingInProgress() const;
	Eigen::Isometry3d getMapToOdom() const;
	Eigen::Isometry3d getMapToRangeSensor() const;


private:

	bool lookupTransform(const std::string& target_frame, const std::string& source_frame,
		    const ros::Time& time, Eigen::Isometry3d *transform) const;
	void update(const MapperParameters &p);
	void estimateNormalsIfNeeded(PointCloud *pcl) const;

  bool isMatchingInProgress_ = false;
  PointCloud map_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
  ros::Time lastMeasurementTimestamp_;
  Eigen::Isometry3d mapToOdom_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d odomToRangeSensorPrev_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d mapToRangeSensor_ = Eigen::Isometry3d::Identity();
  MapperParameters params_;
  open3d::pipelines::registration::ICPConvergenceCriteria icpCriteria_;



};

} /* namespace m545_mapping */
