/*
 * Submap.cpp
 *
 *  Created on: Oct 27, 2021
 *      Author: jelavice
 */

#include "m545_volumetric_mapping/Submap.hpp"
#include <algorithm>
#include <numeric>

namespace m545_mapping {

Submap::Submap() {
}

bool Submap::insertScan(const PointCloud &rawScan, const Eigen::Matrix4d &transform) {

}

void Submap::voxelizeAroundPosition(const Eigen::Vector3d &p) {

}

const Eigen::Isometry3d& Submap::getMapToSubmap() const {
	return mapToSubmap_;
}
const Submap::PointCloud& Submap::getMap() const {
	return map_;
}
void Submap::setMapToSubmap(const Eigen::Isometry3d &T) {
	mapToSubmap_ = T;
}

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
SubmapCollection::SubmapCollection() {
	submaps_.reserve(100);
}

void SubmapCollection::setMapToRangeSensor(const Eigen::Isometry3d &T) {
	mapToRangeSensor_ = T;
}
void SubmapCollection::updateSubmapCollection() {

	if (submaps_.empty()) {
		createNewSubmap(mapToRangeSensor_);
		return;
	}

	const size_t closestMapIdx = findClosestSubmap(mapToRangeSensor_);
	Eigen::Vector3d submapPosition = submaps_.at(closestMapIdx).getMapToSubmap().translation();
	const bool isWithinRange = (mapToRangeSensor_.translation() - submapPosition).norm() < 1e6;
	//todo fix magic
	//todo ensure min num scans in each submap
	if (isWithinRange) {
		activeSubmapIdx_ = closestMapIdx;
	} else {
		createNewSubmap(mapToRangeSensor_);
	}

}

void SubmapCollection::createNewSubmap(const Eigen::Isometry3d &mapToSubmap) {
	Submap newSubmap;
	newSubmap.setMapToSubmap(mapToSubmap);
	submaps_.emplace_back(std::move(newSubmap));
	activeSubmapIdx_ = submaps_.size() - 1;
}

size_t SubmapCollection::findClosestSubmap(const Eigen::Isometry3d &mapToRangeSensor) const {

	std::vector<size_t> idxs(submaps_.size());
	std::iota(idxs.begin(), idxs.end(), 0);
	auto lessThan = [this, &mapToRangeSensor](size_t idxa, size_t idxb) {
		const auto p0 = mapToRangeSensor.translation();
		const auto pa = submaps_.at(idxa).getMapToSubmap().translation();
		const auto pb = submaps_.at(idxb).getMapToSubmap().translation();
		return (p0 - pa).norm() < (p0 - pb).norm();
	};
	return *(std::min_element(idxs.begin(), idxs.end(), lessThan));
}

const Submap& SubmapCollection::getActiveSubmap() const {
	return submaps_.at(activeSubmapIdx_);
}

bool SubmapCollection::insertScan(const PointCloud &rawScan, const Eigen::Matrix4d &transform){

}


} // namespace m545_mapping
