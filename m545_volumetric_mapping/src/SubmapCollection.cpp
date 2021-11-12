/*
 * SubmapCollection.cpp
 *
 *  Created on: Nov 10, 2021
 *      Author: jelavice
 */

#include "m545_volumetric_mapping/SubmapCollection.hpp"
#include "m545_volumetric_mapping/helpers.hpp"
#include "m545_volumetric_mapping/assert.hpp"

#include <algorithm>
#include <numeric>
#include <utility>

namespace m545_mapping {

SubmapCollection::SubmapCollection() {
	submaps_.reserve(100);
	createNewSubmap(mapToRangeSensor_);
}

void SubmapCollection::setMapToRangeSensor(const Transform &T) {
	mapToRangeSensor_ = T;
}

bool SubmapCollection::isEmpty() const {
	return submaps_.empty();
}

const SubmapCollection::Submaps& SubmapCollection::getSubmaps() const {
	return submaps_;
}

size_t SubmapCollection::getTotalNumPoints() const {
	const int nSubmaps = submaps_.size();
	return std::accumulate(submaps_.begin(), submaps_.end(), 0, [](size_t sum, const Submap &s) {
		return sum + s.getMap().points_.size();
	});
}

void SubmapCollection::updateActiveSubmap(const Transform &mapToRangeSensor) {
	if (numScansMergedInActiveSubmap_ < params_.submaps_.minNumRangeData_) {
		return;
	}
	const size_t closestMapIdx = findClosestSubmap(mapToRangeSensor_);
	const auto &closestSubmap = submaps_.at(closestMapIdx);
	const auto &activeSubmap = submaps_.at(activeSubmapIdx_);
	const Eigen::Vector3d closestSubmapPosition = closestSubmap.getMapToSubmapCenter();
	const bool isAnotherSubmapWithinRange = (mapToRangeSensor_.translation() - closestSubmapPosition).norm()
			< params_.submaps_.radius_;
	if (isAnotherSubmapWithinRange) {
		if (closestMapIdx == activeSubmapIdx_) {
			return;
		}
		if (adjacencyMatrix_.isAdjacent(closestSubmap.getId(), activeSubmap.getId())) {
			activeSubmapIdx_ = closestMapIdx;
		} else {
			const bool isTraveledSufficientDistance = (mapToRangeSensor_.translation()
					- activeSubmap.getMapToSubmapCenter()).norm() > params_.submaps_.radius_;
			if (isTraveledSufficientDistance) {
				createNewSubmap(mapToRangeSensor_);
			}
		}
	} else {
		createNewSubmap(mapToRangeSensor_);
	}
}

void SubmapCollection::createNewSubmap(const Transform &mapToSubmap) {
	Submap newSubmap(submapId_++);
	newSubmap.setMapToSubmapOrigin(mapToSubmap);
	newSubmap.setParameters(params_);
	submaps_.emplace_back(std::move(newSubmap));
	activeSubmapIdx_ = submaps_.size() - 1;
	numScansMergedInActiveSubmap_ = 0;
	std::cout << "Created submap: " << activeSubmapIdx_ << std::endl;
}

size_t SubmapCollection::findClosestSubmap(const Transform &mapToRangeSensor) const {

	std::vector<size_t> idxs(submaps_.size());
	std::iota(idxs.begin(), idxs.end(), 0);
	auto lessThan = [this, &mapToRangeSensor](size_t idxa, size_t idxb) {
		const auto p0 = mapToRangeSensor.translation();
		const auto pa = submaps_.at(idxa).getMapToSubmapCenter();
		const auto pb = submaps_.at(idxb).getMapToSubmapCenter();
		return (p0 - pa).norm() < (p0 - pb).norm();
	};
	return *(std::min_element(idxs.begin(), idxs.end(), lessThan));
}

const Submap& SubmapCollection::getActiveSubmap() const {
	return submaps_.at(activeSubmapIdx_);
}

bool SubmapCollection::insertScan(const PointCloud &rawScan, const PointCloud &preProcessedScan,
		const Transform &mapToRangeSensor, const Time &timestamp) {

	mapToRangeSensor_ = mapToRangeSensor;
	if (submaps_.empty()) {
		createNewSubmap(mapToRangeSensor_);
		submaps_.at(activeSubmapIdx_).insertScan(rawScan, preProcessedScan, mapToRangeSensor, timestamp, true);
		++numScansMergedInActiveSubmap_;
		return true;
	}
	const size_t prevActiveSubmapIdx = activeSubmapIdx_;
	updateActiveSubmap(mapToRangeSensor);
	// either different one is active or new one is created
	const bool isActiveSubmapChanged = prevActiveSubmapIdx != activeSubmapIdx_;
	if (isActiveSubmapChanged) {
		std::lock_guard<std::mutex> lck(featureComputationMutex_);
		submaps_.at(prevActiveSubmapIdx).insertScan(rawScan, preProcessedScan, mapToRangeSensor, timestamp, true);
		submaps_.at(prevActiveSubmapIdx).computeSubmapCenter();
		isFinishedSubmap_ = true;
		std::cout << "Active submap changed from " << prevActiveSubmapIdx << " to " << activeSubmapIdx_ << "\n";
		lastFinishedSubmapIdx_ = prevActiveSubmapIdx;
		numScansMergedInActiveSubmap_ = 0;
		const auto id1 = submaps_.at(prevActiveSubmapIdx).getId();
		const auto id2 = submaps_.at(activeSubmapIdx_).getId();
		adjacencyMatrix_.addEdge(id1, id2);
		std::cout << "Adding edge between " << id1 << " and " << id2 << std::endl;
	}
	submaps_.at(activeSubmapIdx_).insertScan(rawScan, preProcessedScan, mapToRangeSensor, timestamp, true);
	++numScansMergedInActiveSubmap_;
	return true;
}

void SubmapCollection::setParameters(const MapperParameters &p) {
	params_ = p;
	for (auto &submap : submaps_) {
		submap.setParameters(p);
	}
	placeRecognition_.setParameters(p);
}

bool SubmapCollection::isFinishedSubmap() const {
	return isFinishedSubmap_;
}

bool SubmapCollection::isBuildingLoopClosureConstraints() const {
	return isBuildingLoopClosureConstraints_;
}

void SubmapCollection::computeFeaturesInLastFinishedSubmap() {
	std::lock_guard<std::mutex> lck(featureComputationMutex_);
	Timer t("feature computation");

	isFinishedSubmap_ = false;
	std::cout << "computing features for submap: " << lastFinishedSubmapIdx_ << std::endl;
	submaps_.at(lastFinishedSubmapIdx_).computeFeatures();
}

void SubmapCollection::buildLoopClosureConstraints() {

	std::lock_guard<std::mutex> lck(loopClosureConstraintMutex_);
	isBuildingLoopClosureConstraints_ = true;

	constraints_ = placeRecognition_.buildLoopClosureConstraints(mapToRangeSensor_, *this, adjacencyMatrix_,
			lastFinishedSubmapIdx_, activeSubmapIdx_);
	isBuildingLoopClosureConstraints_ = false;
}

std::vector<Constraint> SubmapCollection::getAndClearConstraints() {
	std::lock_guard<std::mutex> lck(constraintBuildMutex_);
	auto copy = constraints_;
	constraints_.clear();
	return copy;
}

const std::vector<Constraint>& SubmapCollection::getConstraints() const {
	return constraints_;
}
void SubmapCollection::clearConstraints() {
	std::lock_guard<std::mutex> lck(constraintBuildMutex_);
	constraints_.clear();
}

} // namespace m545_mapping
