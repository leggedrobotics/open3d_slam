/*
 * Odometry.hpp
 *
 *  Created on: Sep 2, 2021
 *      Author: jelavice
 */

#pragma once

#include <open3d/pipelines/registration/Registration.h>
#include <open3d/geometry/PointCloud.h>
#include <ros/time.h>
#include <Eigen/Dense>
#include "m545_volumetric_mapping/Parameters.hpp"


namespace m545_mapping {
class LidarOdometry {

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	LidarOdometry();
	~LidarOdometry() = default;

	bool addRangeScan(const open3d::geometry::PointCloud &cloud, const ros::Time &timestamp);
	const Eigen::Isometry3d &getOdomToRangeSensor() const;
	const open3d::geometry::PointCloud &getPreProcessedCloud() const;
	void setParameters (const OdometryParameters &p);


private:


	open3d::geometry::PointCloud cloudPrev_;
	Eigen::Isometry3d odomToRangeSensor_ = Eigen::Isometry3d::Identity();
	OdometryParameters params_;
	std::shared_ptr<open3d::pipelines::registration::TransformationEstimation> icpObjective_;
	open3d::pipelines::registration::ICPConvergenceCriteria icpConvergenceCriteria_;

};

} // namespace m545_mapping
