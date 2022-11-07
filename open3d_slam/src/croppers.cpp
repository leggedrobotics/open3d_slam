/*
 * croppers.cpp
 *
 *  Created on: Oct 24, 2021
 *      Author: jelavice
 */

#include <Eigen/Core>
#include <vector>
#include "open3d_slam/croppers.hpp"
#include "open3d_slam/typedefs.hpp"

#include "open3d_slam/Parameters.hpp"
#include <utility>
#include <iostream>
#include <numeric>
#ifdef open3d_slam_OPENMP_FOUND
#include <omp.h>
#endif

namespace o3d_slam {

std::unique_ptr<CroppingVolume> croppingVolumeFactory(const ScanCroppingParameters &p) {
	return croppingVolumeFactory(cropperNames.at(p.cropperName_),p);
}
std::unique_ptr<CroppingVolume> croppingVolumeFactory(CroppingVolumeEnum type,  const ScanCroppingParameters &p) {
	switch (type) {
	case CroppingVolumeEnum::Cylinder: {
		auto cropper = std::make_unique<CylinderCroppingVolume>(p.croppingMaxRadius_, p.croppingMinZ_, p.croppingMaxZ_);
		return std::move(cropper);
	}
	case CroppingVolumeEnum::MinRadius: {
		auto cropper = std::make_unique<MinRadiusCroppingVolume>(p.croppingMinRadius_);
		return std::move(cropper);
	}
	case CroppingVolumeEnum::MaxRadius: {
		auto cropper = std::make_unique<MaxRadiusCroppingVolume>(p.croppingMaxRadius_);
		return std::move(cropper);
	}
	case CroppingVolumeEnum::MinMaxRadius: {
		auto cropper = std::make_unique<MinMaxRadiusCroppingVolume>(p.croppingMinRadius_,p.croppingMaxRadius_);
		return std::move(cropper);
	}
	default:
		throw std::runtime_error("Unknown cropper type");
	}
}

bool CroppingVolume::isWithinVolumeImpl(const Eigen::Vector3d &p) const {
	return true;
}

bool CroppingVolume::isWithinVolume(const Eigen::Vector3d &p) const {
  return isInvertVolume_ ? !isWithinVolumeImpl(p) : isWithinVolumeImpl(p);
}

void CroppingVolume::setIsInvertVolume(bool val){
  isInvertVolume_ = val;
}


void CroppingVolume::setPose(const Eigen::Isometry3d &pose) {
	pose_ = pose;
}

CroppingVolume::Indices CroppingVolume::getIndicesWithinVolume(const PointCloud &cloud) const {
	Indices idxs;
	idxs.reserve(cloud.points_.size());
	for (size_t i = 0; i < cloud.points_.size(); ++i) {
		if (isWithinVolume(cloud.points_[i])) {
			idxs.push_back(i);
		}
	}
	return idxs;
}

std::shared_ptr<CroppingVolume::PointCloud> CroppingVolume::crop(const PointCloud &cloud) const {
	std::shared_ptr<CroppingVolume::PointCloud> cropped(new PointCloud());
	const int nPoints = cloud.points_.size();
	cropped->points_.reserve(nPoints);
	if (cloud.HasColors()) {
		cropped->colors_.reserve(nPoints);
	}
	if (cloud.HasNormals()) {
		cropped->normals_.reserve(nPoints);
	}
	if (cloud.HasCovariances()) {
		cropped->covariances_.reserve(nPoints);
	}

	for (size_t i = 0; i < nPoints; ++i) {
		if (isWithinVolume(cloud.points_[i])) {
			cropped->points_.push_back(cloud.points_[i]);
			if (cloud.HasColors()) {
				cropped->colors_.push_back(cloud.colors_[i]);
			}
			if (cloud.HasNormals()) {
				cropped->normals_.push_back(cloud.normals_[i]);
			}
			if (cloud.HasCovariances()) {
				cropped->covariances_.push_back(cloud.covariances_[i]);
			}
		} // end if
	}

	return cropped;

}

void CroppingVolume::crop(PointCloud *cloud) const {
	//todo improve and speed up
	auto cropped = crop(*cloud);
	*cloud = std::move(*cropped);
}

void CroppingVolume::setScaling(double scaling){
  //nothing by default
}

//////////
MinMaxRadiusCroppingVolume::MinMaxRadiusCroppingVolume(double radiusMin, double radiusMax) :
		radiusMin_(radiusMin), radiusMax_(radiusMax) {

}

bool MinMaxRadiusCroppingVolume::isWithinVolumeImpl(const Eigen::Vector3d &p) const {
	const double d = (p - pose_.translation()).norm();
	return  d <= radiusMax_ && d >= radiusMin_;
}
void MinMaxRadiusCroppingVolume::setParameters(double radiusMin, double radiusMax) {
	radiusMin_ = radiusMin;
	radiusMax_ = radiusMax;
}

///////////////////////////////////////////////
///////////////////////////////////////////////
///////////////////////////////////////////////

MaxRadiusCroppingVolume::MaxRadiusCroppingVolume(double radius) :
		radius_(radius) {
}

bool MaxRadiusCroppingVolume::isWithinVolumeImpl(const Eigen::Vector3d &p) const {
	return (p - pose_.translation()).norm() <= radius_;
}
void MaxRadiusCroppingVolume::setParameters(double radius) {
	radius_ = radius;
}

///////////////////////////////////////////////
///////////////////////////////////////////////
///////////////////////////////////////////////

MinRadiusCroppingVolume::MinRadiusCroppingVolume(double radius) :
		radius_(radius) {
}

bool MinRadiusCroppingVolume::isWithinVolumeImpl(const Eigen::Vector3d &p) const {
	return (p - pose_.translation()).norm() >= radius_;
}

void MinRadiusCroppingVolume::setParameters(double radius) {
	radius_ = radius;
}

///////////////////////////////////////////////
///////////////////////////////////////////////
///////////////////////////////////////////////

CylinderCroppingVolume::CylinderCroppingVolume(double radius, double minZ, double maxZ) :
		radius_(radius), minZ_(minZ), maxZ_(maxZ) {

}

bool CylinderCroppingVolume::isWithinVolumeImpl(const Eigen::Vector3d &p) const {
	return p.z() >= minZ_ && p.z() <= maxZ_ && (p - pose_.translation()).head<2>().norm() <= radius_;
}

void CylinderCroppingVolume::setParameters(double radius, double minZ, double maxZ) {
	radius_ = radius;
	minZ_ = minZ;
	maxZ_ = maxZ;
}



///////////////////////////////////////////////
///////////////////////////////////////////////
///////////////////////////////////////////////
bool ColorRangeCropper::isValidColor(const Eigen::Vector3d &c) const {

	return (rgbMin_.array() <= c.array()).all() && (c.array() <= rgbMax_.array()).all();

}

void ColorRangeCropper::setMinBounds(const Eigen::Vector3d &rgbMin){
	rgbMin_ = rgbMin;
}
void ColorRangeCropper::setMaxBounds(const Eigen::Vector3d &rgbMax){
	rgbMax_ = rgbMax;
}

ColorRangeCropper::Indices ColorRangeCropper::getIndicesWithValidColor(const PointCloud &cloud) const {
	if (!cloud.HasColors()) {
		Indices idxs(cloud.points_.size(), 0);
		std::iota(idxs.begin(), idxs.end(), 0);
		return idxs;
	}
	Indices idxs;
	idxs.reserve(cloud.points_.size());
	for (size_t i = 0; i < cloud.points_.size(); ++i) {
		if (isValidColor(cloud.colors_[i])) {
			idxs.push_back(i);
		}
	}
	return idxs;
}

std::shared_ptr<PointCloud> ColorRangeCropper::crop(const PointCloud &cloud) const {
	std::shared_ptr<CroppingVolume::PointCloud> cropped(new PointCloud());

	const int nPoints = cloud.points_.size();
	if (cloud.HasColors()) {
		cropped->colors_.reserve(nPoints);
	} else {
		*cropped = cloud;
		return cropped;
	}

	cropped->points_.reserve(nPoints);

	if (cloud.HasNormals()) {
		cropped->normals_.reserve(nPoints);
	}
	if (cloud.HasCovariances()) {
		cropped->covariances_.reserve(nPoints);
	}

	for (size_t i = 0; i < nPoints; ++i) {
		if (isValidColor(cloud.colors_[i])) {
			cropped->points_.push_back(cloud.points_[i]);
			cropped->colors_.push_back(cloud.colors_[i]);
			if (cloud.HasNormals()) {
				cropped->normals_.push_back(cloud.normals_[i]);
			}
			if (cloud.HasCovariances()) {
				cropped->covariances_.push_back(cloud.covariances_[i]);
			}
		} // end if
	}

	return cropped;
}
void ColorRangeCropper::crop(PointCloud *cloud) const {
	//todo improve and speed up
	auto cropped = crop(*cloud);
	*cloud = std::move(*cropped);
}

} // namespace o3d_slam
