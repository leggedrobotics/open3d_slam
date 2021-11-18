/*
 * SubmapCollection.cpp
 *
 *  Created on: Nov 10, 2021
 *      Author: jelavice
 */

#include "m545_volumetric_mapping/SubmapCollection.hpp"
#include "m545_volumetric_mapping/helpers.hpp"
#include "m545_volumetric_mapping/assert.hpp"
#include <open3d/io/PointCloudIO.h>

#include <algorithm>
#include <numeric>
#include <utility>

namespace m545_mapping {

SubmapCollection::SubmapCollection(std::shared_ptr<OptimizationProblem> optimization) :
		optimization_(optimization) {
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
	if (submaps_.size() > 1) {
		const auto c = buildOdometryConstraint(activeSubmapIdx_ - 1, activeSubmapIdx_);
		optimization_->addOdometryConstraint(c);
	}
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
		std::cout << "Active submap changed from " << prevActiveSubmapIdx << " to " << activeSubmapIdx_ << "\n";
		lastFinishedSubmapIdx_ = prevActiveSubmapIdx;
		finishedSubmapsIdxs_.push(prevActiveSubmapIdx);
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

bool SubmapCollection::isBuildingLoopClosureConstraints() const {
	return isBuildingLoopClosureConstraints_;
}

void SubmapCollection::computeFeatures() {
	std::lock_guard<std::mutex> lck(featureComputationMutex_);
	isComputingFeatures_ = true;
	Timer t("feature computation");
	const auto submapsIdxsForFeatureComputation = finishedSubmapsIdxs_.popAllElements();
	for (const auto &id : submapsIdxsForFeatureComputation) {
		std::cout << "computing features for submap: " << id << std::endl;
		submaps_.at(id).computeFeatures();
		loopClosureCandidatesIdxs_.push(id);
	}
	// if the features inthose maps are computed, they can be considered for a loop closure

	isComputingFeatures_ = false;
}

bool SubmapCollection::isComputingFeatures() const {
	return isComputingFeatures_;
}

const ThreadSafeBuffer<SubmapCollection::SubmapId>& SubmapCollection::getFinishedSubmapIds() const {
	return finishedSubmapsIdxs_;
}

const ThreadSafeBuffer<SubmapCollection::SubmapId>& SubmapCollection::getLoopClosureCandidateIds() const {
	return loopClosureCandidatesIdxs_;
}

void SubmapCollection::buildLoopClosureConstraints() {
	std::lock_guard<std::mutex> lck(loopClosureConstraintMutex_);
	isBuildingLoopClosureConstraints_ = true;
	Timer t("loop closing");

	const auto loopClosureCandidatesIdxs = loopClosureCandidatesIdxs_.popAllElements();
	for (const auto &id : loopClosureCandidatesIdxs) {
		const auto constraints = placeRecognition_.buildLoopClosureConstraints(mapToRangeSensor_, *this,
				adjacencyMatrix_, id, activeSubmapIdx_);
		if (!constraints.empty()) {
			for (int i = 0; i < constraints.size(); ++i) {
				optimization_->addLoopClosureConstraint(constraints.at(i));
			}
			std::cout << " building loop closure constraints for submap: " << id << " resulted in: "
					<< constraints.size() << " new constraints \n";
		}
	}

	isBuildingLoopClosureConstraints_ = false;
}

Constraint SubmapCollection::buildOdometryConstraint(size_t sourceSubmapIdx, size_t targetSubmapIdx) const {

	Constraint c;
	c.sourceSubmapIdx_ = sourceSubmapIdx;
	c.targetSubmapIdx_ = targetSubmapIdx;
	const Transform &mapToSource = submaps_.at(sourceSubmapIdx).getMapToSubmapOrigin();
	const Transform &mapToTarget = submaps_.at(targetSubmapIdx).getMapToSubmapOrigin();
	c.sourceToTarget_ = mapToSource.inverse() * mapToTarget;
//	std::cout << "odom constraints: \n";
//	std::cout << " source: " << asString(mapToSource) << std::endl;
//	std::cout << " target: " << asString(mapToTarget) << std::endl;
//	std::cout << " source to target: " << asString(c.sourceToTarget_) << std::endl;
//	std::cout << " source transformed into target: \n";
//	std::cout << asString(mapToSource * c.sourceToTarget_) << "\n \n";
	return std::move(c);
}

void SubmapCollection::dumpToFile(const std::string &folderPath, const std::string &filename) const {

	for (size_t i = 0; i < submaps_.size(); ++i) {
		auto copy = submaps_.at(i).getMap();
		const auto optimizedPoses = optimization_->getNodeValues();
		if (i < optimizedPoses.size()) {
			const auto mapToOld = submaps_.at(i).getMapToSubmapOrigin();
			const auto mapToNew = optimizedPoses.at(i).mapToSubmap_;
			const auto deltaT = mapToOld.inverse() * mapToNew;
			copy.Transform(deltaT.matrix());
		}
		//todo handle a case where a submap has been added in the meantime
		const std::string fullPath = folderPath + "/" + filename + "_" + std::to_string(i) + ".pcd";
		open3d::io::WritePointCloudToPCD(fullPath, copy, open3d::io::WritePointCloudOption());
//		std::cout <<"written to: " << fullPath << std::endl;
	}

}

} // namespace m545_mapping
