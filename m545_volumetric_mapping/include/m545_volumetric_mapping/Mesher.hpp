/*
 * Mesher.hpp
 *
 *  Created on: Oct 8, 2021
 *      Author: jelavice
 */

#pragma once

#include <mutex>
#include <open3d/geometry/PointCloud.h>
#include <open3d/geometry/TriangleMesh.h>
#include "m545_volumetric_mapping/Parameters.hpp"
#include <sensor_msgs/PointCloud2.h>


namespace m545_mapping {


class Mesher {

public:

	using PointCloud = open3d::geometry::PointCloud;
	using TriangleMesh = open3d::geometry::TriangleMesh;

	Mesher();
	~Mesher() = default;
	void buildMeshFromCloud(const PointCloud &cloud);
	bool isMeshingInProgress() const;
	void setParameters(const MesherParameters &p);
	const TriangleMesh &getMesh() const;
	void setCurrentPose(const Eigen::Isometry3d &pose);
//    const sensor_msgs::PointCloud2 getCloud(sensor_msgs::PointCloud2 &cloud);
private:
	bool isMeshingInProgress_ = false;
	std::mutex meshingMutex_;
	mutable std::mutex meshingAccessMutex_;
	std::shared_ptr<TriangleMesh> mesh_;
	MesherParameters params_;
	Eigen::Isometry3d currentPose_ = Eigen::Isometry3d::Identity();


};



} // namespace m545_mapping
