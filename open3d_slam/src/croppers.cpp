/*
 * croppers.cpp
 *
 *  Created on: Oct 24, 2021
 *      Author: jelavice
 */

#include <Eigen/Core>
#include <vector>
#include "open3d_slam/croppers.hpp"
#include <utility>
#include <iostream>
#ifdef open3d_slam_OPENMP_FOUND
#include <omp.h>
#endif

namespace o3d_slam {

std::unique_ptr<CroppingVolume> croppingVolumeFactory(const std::string &type, double radius, double minZ,
		double maxZ) {
	return croppingVolumeFactory(cropperNames.at(type), radius, minZ, maxZ);
}
std::unique_ptr<CroppingVolume> croppingVolumeFactory(CroppingVolumeEnum type, double radius, double minZ,
		double maxZ) {
	switch (type) {
	case CroppingVolumeEnum::Cylinder: {
		auto cropper = std::make_unique<CylinderCroppingVolume>(radius, minZ, maxZ);
		return std::move(cropper);
	}
	case CroppingVolumeEnum::MinRadius: {
		auto cropper = std::make_unique<MinRadiusCroppingVolume>(radius);
		return std::move(cropper);
	}
	case CroppingVolumeEnum::MaxRadius: {
		auto cropper = std::make_unique<MaxRadiusCroppingVolume>(radius);
		return std::move(cropper);
	}
	default:
		throw std::runtime_error("Unknown cropper type");
	}
}

bool CroppingVolume::isWithinVolume(const Eigen::Vector3d &p) const {
	return true;
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

std::shared_ptr<CroppingVolume::PointCloud> CroppingVolume::cropMultiThreaded(const PointCloud &cloud) const {

	std::shared_ptr<CroppingVolume::PointCloud> cropped(new PointCloud());

	const int nPoints = cloud.points_.size();
	cropped->points_.reserve(nPoints);
	if (cloud.HasColors()) {
		cropped->colors_.reserve(nPoints);
	}
	if (cloud.HasNormals()) {
		cropped->normals_.reserve(nPoints);
	}

#ifndef open3d_slam_OPENMP_FOUND
	std::cerr << "OpemMP not found, defaulting to single threaded implementation\n";
#else
#pragma omp parallel for
#endif
	for (size_t i = 0; i < nPoints; ++i) {
		if (isWithinVolume(cloud.points_[i])) {
#ifdef open3d_slam_OPENMP_FOUND
#pragma omp critical
#endif
			{
				cropped->points_.push_back(cloud.points_[i]);
				if (cloud.HasColors()) {
					cropped->colors_.push_back(cloud.colors_[i]);
				}
				if (cloud.HasNormals()) {
					cropped->normals_.push_back(cloud.normals_[i]);
				}
			} // end pragma
		} // end if
	}

	return cropped;
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

	for (size_t i = 0; i < nPoints; ++i) {
		if (isWithinVolume(cloud.points_[i])) {
			cropped->points_.push_back(cloud.points_[i]);
			if (cloud.HasColors()) {
				cropped->colors_.push_back(cloud.colors_[i]);
			}
			if (cloud.HasNormals()) {
				cropped->normals_.push_back(cloud.normals_[i]);
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

///////////////////////////////////////////////
///////////////////////////////////////////////
///////////////////////////////////////////////

MaxRadiusCroppingVolume::MaxRadiusCroppingVolume(double radius) :
		radius_(radius) {
}

bool MaxRadiusCroppingVolume::isWithinVolume(const Eigen::Vector3d &p) const {
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

bool MinRadiusCroppingVolume::isWithinVolume(const Eigen::Vector3d &p) const {
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

bool CylinderCroppingVolume::isWithinVolume(const Eigen::Vector3d &p) const {
	return p.z() >= minZ_ && p.z() <= maxZ_ && (p - pose_.translation()).head<2>().norm() <= radius_;
}

void CylinderCroppingVolume::setParameters(double radius, double minZ, double maxZ) {
	radius_ = radius;
	minZ_ = minZ;
	maxZ_ = maxZ;
}

} // namespace o3d_slam
