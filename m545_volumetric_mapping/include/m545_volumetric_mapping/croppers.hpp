/*
 * croppers.hpp
 *
 *  Created on: Oct 24, 2021
 *      Author: jelavice
 */

#pragma once

#include <Eigen/Dense>
#include <vector>
#include <open3d/geometry/PointCloud.h>

namespace m545_mapping {

class Cropper {

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	using PointCloud = open3d::geometry::PointCloud;
	using Indices =std::vector<size_t>;
	Cropper() = default;
	virtual ~Cropper() = default;


	void setPose(const Eigen::Isometry3d &pose);
	virtual bool isWithinVolume(const Eigen::Vector3d &p) const;
	Indices getIndicesWithinVolume(const PointCloud &cloud) const;
	std::shared_ptr<PointCloud> crop(const PointCloud &cloud) const;

protected:

	Eigen::Isometry3d pose_=Eigen::Isometry3d::Identity();

};

class BallCropper : public Cropper{
public:
	BallCropper(double radius);
	virtual ~BallCropper() = default;

	bool isWithinVolume(const Eigen::Vector3d &p) const final;


private:
	double radius_=1e6;

};


} // namespace m545_mapping
