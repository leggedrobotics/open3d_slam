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

bool CroppingVolume::isWithinVolume(const Eigen::Vector3d &p) const {
	return true;
}

void CroppingVolume::setPose(const Eigen::Isometry3d &pose){
	pose_ = pose;
}

CroppingVolume::Indices CroppingVolume::getIndicesWithinVolume(const PointCloud &cloud) const{

	Indices idxs;
	idxs.reserve(cloud.points_.size());

	for(size_t i =0; i < cloud.points_.size(); ++i){
		if(isWithinVolume(cloud.points_[i])){
			idxs.push_back(i);
		}
	}

	return idxs;
}

std::shared_ptr<CroppingVolume::PointCloud> CroppingVolume::crop(const PointCloud &cloud) const{
	const auto idxsInside = getIndicesWithinVolume(cloud);
	return cloud.SelectByIndex(idxsInside);
}

void CroppingVolume::crop(PointCloud *cloud) const{
	//todo improve and speed up
	auto cropped = crop(*cloud);
	*cloud = std::move(*cropped);
}

///////////////////////////////////////////////
///////////////////////////////////////////////
///////////////////////////////////////////////

MaxRadiusCroppingVolume::MaxRadiusCroppingVolume(double radius):radius_(radius){
}

bool MaxRadiusCroppingVolume::isWithinVolume(const Eigen::Vector3d &p) const{
	return (p-pose_.translation()).norm() <= radius_;
}

} // namespace m545_mapping
