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
#include <map>

namespace o3d_slam {

enum class CroppingVolumeEnum : int{
	MaxRadius,
	MinRadius,
	Cylinder
};

static const std::map<std::string, CroppingVolumeEnum> cropperNames{
	{"MaxRadius",CroppingVolumeEnum::MaxRadius},
	{"MinRadius",CroppingVolumeEnum::MinRadius},
	{"Cylinder",CroppingVolumeEnum::Cylinder}
};



class CroppingVolume {

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	using PointCloud = open3d::geometry::PointCloud;
	using Indices =std::vector<size_t>;
	CroppingVolume() = default;
	virtual ~CroppingVolume() = default;


	void setPose(const Eigen::Isometry3d &pose);
	virtual bool isWithinVolume(const Eigen::Vector3d &p) const;
	Indices getIndicesWithinVolume(const PointCloud &cloud) const;
	std::shared_ptr<PointCloud> crop(const PointCloud &cloud) const;
	std::shared_ptr<PointCloud> cropMultiThreaded(const PointCloud &cloud) const;
	void crop(PointCloud *cloud) const;


protected:

	Eigen::Isometry3d pose_=Eigen::Isometry3d::Identity();

};

class MaxRadiusCroppingVolume : public CroppingVolume{
public:
	MaxRadiusCroppingVolume() = default;
	~MaxRadiusCroppingVolume() override= default;
	MaxRadiusCroppingVolume(double radius);
	bool isWithinVolume(const Eigen::Vector3d &p) const final;
	void setParameters(double radius);
private:
	double radius_=1e6;

};

class MinRadiusCroppingVolume : public CroppingVolume{
public:
	MinRadiusCroppingVolume() = default;
	MinRadiusCroppingVolume(double radius);
	~MinRadiusCroppingVolume() override = default;

	bool isWithinVolume(const Eigen::Vector3d &p) const final;
	void setParameters(double radius);

private:
	double radius_=0.0;

};

class CylinderCroppingVolume : public CroppingVolume{
public:
	CylinderCroppingVolume();
	CylinderCroppingVolume(double radius, double minZ, double maxZ);
	~CylinderCroppingVolume() override = default;
	void setParameters(double radius, double minZ, double maxZ);
	bool isWithinVolume(const Eigen::Vector3d &p) const final;


private:
	double radius_=1e6;
	double minZ_ = -1e3;
	double maxZ_ = 1e3;

};

std::unique_ptr<CroppingVolume> croppingVolumeFactory(const std::string &type, double radius, double minZ, double maxZ);
std::unique_ptr<CroppingVolume> croppingVolumeFactory(CroppingVolumeEnum type, double radius, double minZ, double maxZ);

} // namespace o3d_slam
