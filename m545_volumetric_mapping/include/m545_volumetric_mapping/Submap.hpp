/*
 * Submap.hpp
 *
 *  Created on: Oct 27, 2021
 *      Author: jelavice
 */


#pragma once

#include <open3d/geometry/PointCloud.h>
#include <ros/time.h>
#include <Eigen/Dense>
#include "m545_volumetric_mapping/Parameters.hpp"
#include "m545_volumetric_mapping/croppers.hpp"

namespace m545_mapping {

class Submap {

public:
	using PointCloud = open3d::geometry::PointCloud;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	Submap();
	~Submap() = default;

	bool insertScan(const PointCloud &rawScan, const Eigen::Matrix4d &transform);
	void voxelizeAroundPosition(const Eigen::Vector3d &p);
	const Eigen::Isometry3d &getMapToSubmap() const;
	const PointCloud &getMap() const;
	void setMapToSubmap(const Eigen::Isometry3d &T);

private:
	PointCloud map_;
	Eigen::Isometry3d mapToSubmap_ = Eigen::Isometry3d::Identity();
	Eigen::Isometry3d mapToRangeSensor_ = Eigen::Isometry3d::Identity();
	std::shared_ptr<CroppingVolume> scanCropper_;
};



class SubmapCollection{

	SubmapCollection();
	~SubmapCollection() = default;

	using PointCloud = open3d::geometry::PointCloud;
	void setMapToRangeSensor(const Eigen::Isometry3d &T);
	void updateSubmapCollection();
	const Submap &getActiveSubmap() const;
	bool insertScan(const PointCloud &rawScan, const Eigen::Matrix4d &transform);

private:

	void createNewSubmap(const Eigen::Isometry3d &mapToSubmap);
	size_t findClosestSubmap(const Eigen::Isometry3d &mapToRangesensor) const;


	Eigen::Isometry3d mapToRangeSensor_ = Eigen::Isometry3d::Identity();
	std::vector<Submap> submaps_;
	size_t activeSubmapIdx_ = 0;

};

} // namespace m545_mapping
