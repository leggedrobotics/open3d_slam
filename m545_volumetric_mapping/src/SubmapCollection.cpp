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
#include <open3d/pipelines/registration/FastGlobalRegistration.h>
#include <open3d/pipelines/registration/Registration.h>
#include <open3d/io/PointCloudIO.h>

namespace m545_mapping {

namespace {
const double featureVoxelSize = 0.5;
namespace registration = open3d::pipelines::registration;
std::shared_ptr<registration::TransformationEstimation> icpObjective;
} // namespace

SubmapCollection::SubmapCollection() {
	submaps_.reserve(100);
	createNewSubmap(mapToRangeSensor_);
	icpObjective = icpObjectiveFactory(IcpObjective::PointToPlane);
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
	const Eigen::Vector3d closestSubmapPosition = closestSubmap.getMapToSubmap().translation();
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
					- activeSubmap.getMapToSubmap().translation()).norm() > params_.submaps_.radius_;
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
	newSubmap.setMapToSubmap(mapToSubmap);
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
		const auto pa = submaps_.at(idxa).getMapToSubmap().translation();
		const auto pb = submaps_.at(idxb).getMapToSubmap().translation();
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
		submaps_.at(activeSubmapIdx_).insertScan(rawScan, preProcessedScan, mapToRangeSensor, timestamp,true);
		++numScansMergedInActiveSubmap_;
		return true;
	}
	const size_t prevActiveSubmapIdx = activeSubmapIdx_;
	updateActiveSubmap(mapToRangeSensor);
	// either different one is active or new one is created
	const bool isActiveSubmapChanged = prevActiveSubmapIdx != activeSubmapIdx_;
	if (isActiveSubmapChanged) {
		std::lock_guard<std::mutex> lck(featureComputationMutex_);
		submaps_.at(prevActiveSubmapIdx).insertScan(rawScan, preProcessedScan, mapToRangeSensor, timestamp,true);
		submaps_.at(prevActiveSubmapIdx).centerOrigin();
		isFinishedSubmap_ = true;
		std::cout << "Active submap changed from " << prevActiveSubmapIdx << " to " << activeSubmapIdx_ << "\n";
		lastFinishedSubmapIdx_ = prevActiveSubmapIdx;
		numScansMergedInActiveSubmap_ = 0;
		const auto id1 = submaps_.at(prevActiveSubmapIdx).getId();
		const auto id2 = submaps_.at(activeSubmapIdx_).getId();
		adjacencyMatrix_.addEdge(id1, id2);
		std::cout << "Adding edge between " << id1 << " and " << id2 << std::endl;
	}
	submaps_.at(activeSubmapIdx_).insertScan(rawScan, preProcessedScan, mapToRangeSensor, timestamp,true);
	++numScansMergedInActiveSubmap_;
	return true;
}

void SubmapCollection::setParameters(const MapperParameters &p) {
	params_ = p;
	for (auto &submap : submaps_) {
		submap.setParameters(p);
	}
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
	// look at the last submap finished
	// iterate over all submaps, except the active one
	// prune the search!!!!
	// do the fast matching
	// refine with icp
	Timer t("loop closing");
	using namespace open3d::pipelines::registration;
	const auto edgeLengthChecker = CorrespondenceCheckerBasedOnEdgeLength(0.5);
	const auto distanceChecker = CorrespondenceCheckerBasedOnDistance(1.5 * featureVoxelSize);
	const size_t lastFinishedSubmapIdx = lastFinishedSubmapIdx_;
	const size_t activeSubmapIdx = activeSubmapIdx_;
	const auto &lastBuiltSubmap = submaps_.at(lastFinishedSubmapIdx);
	const auto closeSubmapsIdxs = std::move(
			getCloseSubmapsIdxs(lastBuiltSubmap.getMapToSubmap(), lastFinishedSubmapIdx, activeSubmapIdx));
	std::cout << "considering submap " << lastFinishedSubmapIdx << " for loop closure, candidate submaps: "
			<< closeSubmapsIdxs.size() << std::endl;
	const auto source = lastBuiltSubmap.getSparseMap();
	const auto sourceFeature = lastBuiltSubmap.getFeatures();
	const Time lastBuiltSubmapFinishTime_ = lastBuiltSubmap.getCreationTime();
	for (const auto id : closeSubmapsIdxs) {
		std::cout << "matching submap: " << lastFinishedSubmapIdx << " with submap: " << id << "\n";
		const auto &candidateSubmap = submaps_.at(id);
		const Time candidateSubmapFinishTime = candidateSubmap.getCreationTime();
		const auto target = candidateSubmap.getSparseMap();
		const auto targetFeature = candidateSubmap.getFeatures();
		RegistrationResult ransacResult = RegistrationRANSACBasedOnFeatureMatching(source, target, sourceFeature,
				targetFeature, true, featureVoxelSize * 1.5, TransformationEstimationPointToPoint(false), 3, {
						distanceChecker, edgeLengthChecker }, RANSACConvergenceCriteria(1000000, 0.99));

		if (ransacResult.correspondence_set_.size() >= 25) {
			std::cout << "source features num: " << sourceFeature.Num() << "\n";
			std::cout << "target features num: " << targetFeature.Num() << "\n";
			std::cout << "num points source: " << source.points_.size() << "\n";
			std::cout << "num points target: " << target.points_.size() << "\n";
			std::cout << "registered num correspondences: " << ransacResult.correspondence_set_.size() << std::endl;
			std::cout << "registered with fitness: " << ransacResult.fitness_ << std::endl;
			std::cout << "registered with rmse: " << ransacResult.inlier_rmse_ << std::endl;
			std::cout << "registered with transformation: \n" << asString(Transform(ransacResult.transformation_))
					<< std::endl;
			registration::ICPConvergenceCriteria icpCriteria;
			icpCriteria.max_iteration_ = 40;
			const auto icpResult = open3d::pipelines::registration::RegistrationICP(source, target,
					featureVoxelSize * 3.0, ransacResult.transformation_, *icpObjective, icpCriteria);
			std::cout << "refined with fitness: " << icpResult.fitness_ << std::endl;
			std::cout << "refined with rmse: " << icpResult.inlier_rmse_ << std::endl;
			std::cout << "refined with transformation: \n" << asString(Transform(icpResult.transformation_))
					<< std::endl;
//			const std::string folder =
//					"/home/jelavice/catkin_workspaces/open3d_ws/src/m545_volumetric_mapping/m545_volumetric_mapping/data/";
//			open3d::io::WritePointCloudToPCD(folder + "target.pcd", target, open3d::io::WritePointCloudOption());
//			open3d::io::WritePointCloudToPCD(folder + "source.pcd", source, open3d::io::WritePointCloudOption());
//			auto registered = source.Transform(icpResult.transformation_);
//
//			open3d::io::WritePointCloudToPCD(folder + "source_registered.pcd", registered,
//					open3d::io::WritePointCloudOption());
			Constraint c;
			c.relativeTransformation_ = Transform(icpResult.transformation_);
			c.timeBegin_ = candidateSubmapFinishTime;
			c.timeFinish_ = lastBuiltSubmapFinishTime_;
			c.submapIdxBegin_ = id;
			c.submapIdxBegin_ = lastFinishedSubmapIdx;
			std::lock_guard<std::mutex> lck(constraintBuildMutex_);
			constraints_.emplace_back(std::move(c));
			//todo not sure is this a valid assumption
//			assert_ge(toUniversal(c.timeFinish_), toUniversal(c.timeBegin_));
		}
	}
	isBuildingLoopClosureConstraints_ = false;
}

std::vector<size_t> SubmapCollection::getCloseSubmapsIdxs(const Transform &mapToRangeSensor,
		size_t lastFinishedSubmapIdx, size_t currentActiveSubmapIdx) const {
	std::vector<size_t> idxs;
	const size_t nSubmaps = submaps_.size();
	idxs.reserve(nSubmaps);
	for (size_t i = 0; i < nSubmaps; ++i) {
		if (i == lastFinishedSubmapIdx || i == currentActiveSubmapIdx) {
			continue;
		}

		const double distance =
				(mapToRangeSensor_.translation() - submaps_.at(i).getMapToSubmap().translation()).norm();
		const bool isTooFar = distance > params_.submaps_.radius_;
//		std::cout << "distance submap to submap " << i << " : " << distance << std::endl;
		if (isTooFar) {
			continue;
		}

		idxs.push_back(i);
	}
	return idxs;
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
