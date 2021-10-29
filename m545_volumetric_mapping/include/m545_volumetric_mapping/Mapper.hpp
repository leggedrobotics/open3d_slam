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
#include "m545_volumetric_mapping/time.hpp"
#include "m545_volumetric_mapping/croppers.hpp"
#include "m545_volumetric_mapping/Submap.hpp"


namespace m545_mapping {

class Mapper {

public:

	using PointCloud = open3d::geometry::PointCloud;

	Mapper();
	~Mapper() = default;

	const PointCloud& getMap() const;
	const PointCloud& getDenseMap() const;
	const Submap& getActiveSubmap() const;
	const SubmapCollection &getSubmaps() const;
	PointCloud getAssembledMap() const;
	void addRangeMeasurement(const PointCloud &cloud, const ros::Time &timestamp);
	void setParameters(const MapperParameters &p);
	bool isMatchingInProgress() const;
	bool isManipulatingMap() const;

	Eigen::Isometry3d getMapToOdom() const;
	Eigen::Isometry3d getMapToRangeSensor() const;


private:
	std::shared_ptr<PointCloud> preProcessScan(const PointCloud &scan) const;
	void update(const MapperParameters &p);
	void estimateNormalsIfNeeded(PointCloud *pcl) const;


	bool isMatchingInProgress_ = false;
	bool isManipulatingMap_ = false;
	tf2_ros::Buffer tfBuffer_;
	tf2_ros::TransformListener tfListener_;
	ros::Time lastMeasurementTimestamp_;
	Eigen::Isometry3d mapToOdom_ = Eigen::Isometry3d::Identity();
	Eigen::Isometry3d odomToRangeSensorPrev_ = Eigen::Isometry3d::Identity();
	Eigen::Isometry3d mapToRangeSensor_ = Eigen::Isometry3d::Identity();
	MapperParameters params_;
	open3d::pipelines::registration::ICPConvergenceCriteria icpCriteria_;
	std::mutex mapManipulationMutex_;
	std::shared_ptr<CroppingVolume> scanMatcherCropper_;
	std::shared_ptr<CroppingVolume> mapBuilderCropper_;
	SubmapCollection submaps_;

};

} /* namespace m545_mapping */
