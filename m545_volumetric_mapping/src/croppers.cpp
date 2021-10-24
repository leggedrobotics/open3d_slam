/*
 * croppers.cpp
 *
 *  Created on: Oct 24, 2021
 *      Author: jelavice
 */

#include <Eigen/Core>
#include <vector>
#include "m545_volumetric_mapping/croppers.hpp"
#include <utility>

namespace m545_mapping {

bool Cropper::isWithinVolume(const Eigen::Vector3d &p) const {
	return true;
}

void Cropper::setPose(const Eigen::Isometry3d &pose){
	pose_ = pose;
}

Cropper::Indices Cropper::getIndicesWithinVolume(const PointCloud &cloud) const{

	Indices idxs;
	idxs.reserve(cloud.points_.size());

	for(size_t i =0; i < cloud.points_.size(); ++i){
		if(isWithinVolume(cloud.points_[i])){
			idxs.push_back(i);
		}
	}

	return idxs;
}

std::shared_ptr<Cropper::PointCloud> Cropper::crop(const PointCloud &cloud) const{
	const auto idxsInside = getIndicesWithinVolume(cloud);
	return cloud.SelectByIndex(idxsInside);
}

void Cropper::crop(PointCloud *cloud) const{
	//todo improve and speed up
	auto cropped = crop(*cloud);
	*cloud = std::move(*cropped);
}

///////////////////////////////////////////////
///////////////////////////////////////////////
///////////////////////////////////////////////

BallCropper::BallCropper(double radius):radius_(radius){
}

bool BallCropper::isWithinVolume(const Eigen::Vector3d &p) const{
	return (p-pose_.translation()).norm() <= radius_;
}

} // namespace m545_mapping
