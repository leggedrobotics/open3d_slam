/*
 * SubmapCollection.cpp
 *
 *  Created on: Nov 10, 2021
 *      Author: jelavice
 */

#include "open3d_slam/SubmapCollection.hpp"
#include "open3d_slam/helpers.hpp"
#include "open3d_slam/assert.hpp"
#include "open3d_slam/typedefs.hpp"
#include "open3d_slam/math.hpp"
#include "open3d_slam/magic.hpp"
#include "open3d_slam/output.hpp"
#include "open3d_slam/constraint_builders.hpp"

#include <open3d/io/PointCloudIO.h>
#include <open3d/pipelines/registration/Registration.h>

#include <algorithm>
#include <numeric>
#include <utility>
#include <set>
#include <thread>

namespace o3d_slam {

SubmapCollection::SubmapCollection() {
	submaps_.reserve(500);
	createNewSubmap(mapToRangeSensor_);
	overlapScansBuffer_.set_size_limit(5);
}

void SubmapCollection::setMapToRangeSensor(const Transform &T) {
	mapToRangeSensor_ = T;
}

bool SubmapCollection::isEmpty() const {
	return submaps_.empty();
}

Submap* SubmapCollection::getSubmapPtr(SubmapId idx) {
	return &(submaps_.at(idx));
}

const Submap &SubmapCollection::getSubmap(SubmapId idx) const{
	return submaps_.at(idx);
}
size_t SubmapCollection::getNumSubmaps() const{
	return submaps_.size();
}

SubmapCollection::TimestampedSubmapIds SubmapCollection::popFinishedSubmapIds() {
	return finishedSubmapsIdxs_.popAllElements();
}

SubmapCollection::TimestampedSubmapIds SubmapCollection::popLoopClosureCandidates() {
	return loopClosureCandidatesIdxs_.popAllElements();
}

size_t SubmapCollection::numFinishedSubmaps() const {
	return finishedSubmapsIdxs_.size();
}

size_t SubmapCollection::numLoopClosureCandidates() const {
	return loopClosureCandidatesIdxs_.size();
}

size_t SubmapCollection::getTotalNumPoints() const {
	const int nSubmaps = submaps_.size();
	return std::accumulate(submaps_.begin(), submaps_.end(), 0, [](size_t sum, const Submap &s) {
		return sum + s.getMapPointCloud().points_.size();
	});
}

void SubmapCollection::updateAdjacencyMatrix(const Constraints &loopClosureConstraints) {
	for (const auto &c : loopClosureConstraints) {
		adjacencyMatrix_.addEdge(c.sourceSubmapIdx_, c.targetSubmapIdx_);
		adjacencyMatrix_.markAsLoopClosureSubmap(c.sourceSubmapIdx_);
		adjacencyMatrix_.markAsLoopClosureSubmap(c.targetSubmapIdx_);
	}
}

void SubmapCollection::addScanToBuffer(const PointCloud &scan, const Transform &mapToRangeSensor,
		const Time &timestamp) {
	overlapScansBuffer_.push(ScanTimeTransform { scan, timestamp, mapToRangeSensor });
}

void SubmapCollection::insertBufferedScans(Submap *submap) {
	while (!overlapScansBuffer_.empty()) {
		auto scan = overlapScansBuffer_.pop();
		submap->insertScan(scan.cloud_, scan.cloud_, scan.mapToRangeSensor_, scan.timestamp_, false);
	}
}

void SubmapCollection::updateActiveSubmap(const Transform &mapToRangeSensor, const PointCloud &scan) {
	if (isForceNewSubmapCreation_){
		createNewSubmap(mapToRangeSensor_);
		isForceNewSubmapCreation_ = false;
		return;
	}

	if (numScansMergedInActiveSubmap_ < params_.submaps_.minNumRangeData_) {
		return;
	}

	if (params_.isUseInitialMap_){
		return; // do not switch maps if we are doing in the localization mode
	}
	const size_t closestMapIdx = findClosestSubmap(mapToRangeSensor_);
	const Submap &closestSubmap = submaps_.at(closestMapIdx);
	const Submap &activeSubmap = submaps_.at(activeSubmapIdx_);
	const Eigen::Vector3d closestSubmapPosition = closestSubmap.getMapToSubmapCenter();
	const bool isAnotherSubmapWithinRange = (mapToRangeSensor_.translation() - closestSubmapPosition).norm()
			< params_.submaps_.radius_;
	if (isAnotherSubmapWithinRange) {
		if (closestMapIdx == activeSubmapIdx_) {
			return;
		}
		if (adjacencyMatrix_.isAdjacent(closestSubmap.getId(), activeSubmap.getId())
				&& isSwitchingSubmapsConsistant(scan, closestSubmap.getId(), mapToRangeSensor)) {
			//todo here we could put a consistency check
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
	const size_t submapId = submapId_++;
	const size_t submapParentId = activeSubmapIdx_;
	Submap newSubmap(submapId, submapParentId);
	newSubmap.setMapToSubmapOrigin(mapToSubmap);
	newSubmap.setParameters(params_);
	submaps_.emplace_back(std::move(newSubmap));
	activeSubmapIdx_ = submaps_.size() - 1;
	numScansMergedInActiveSubmap_ = 0;
	std::cout << "Created submap: " << activeSubmapIdx_ << " with parent " << submapParentId << std::endl;
//	std::cout << "Submap " << activeSubmapIdx_ << " pose: " << asString(newSubmap.getMapToSubmapOrigin())
//			<< std::endl;
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

void SubmapCollection::forceNewSubmapCreation(){
	if (submaps_.empty()){
		return;
	}
	isForceNewSubmapCreation_ = true;
	insertScan(PointCloud(),PointCloud(), mapToRangeSensor_,timestamp_);
	isForceNewSubmapCreation_ = false;
}

bool SubmapCollection::insertScan(const PointCloud &rawScan, const PointCloud &preProcessedScan,
		const Transform &mapToRangeSensor, const Time &timestamp) {

	mapToRangeSensor_ = mapToRangeSensor;
	timestamp_ = timestamp;
	if (submaps_.empty()) {
		createNewSubmap(mapToRangeSensor_);
		submaps_.at(activeSubmapIdx_).insertScan(rawScan, preProcessedScan, mapToRangeSensor, timestamp, true);
		++numScansMergedInActiveSubmap_;
		return true;
	}
	addScanToBuffer(preProcessedScan, mapToRangeSensor, timestamp);
	const size_t prevActiveSubmapIdx = activeSubmapIdx_;
	updateActiveSubmap(mapToRangeSensor, preProcessedScan);
	// either different one is active or new one is created
	const bool isActiveSubmapChanged = prevActiveSubmapIdx != activeSubmapIdx_;
	if (isActiveSubmapChanged) {
		std::lock_guard<std::mutex> lck(featureComputationMutex_);
		submaps_.at(prevActiveSubmapIdx).insertScan(rawScan, preProcessedScan, mapToRangeSensor, timestamp, true);
		submaps_.at(prevActiveSubmapIdx).computeSubmapCenter();
		std::cout << "Active submap changed from " << prevActiveSubmapIdx << " to " << activeSubmapIdx_ << "\n";
		lastFinishedSubmapIdx_ = prevActiveSubmapIdx;
		TimestampedSubmapId timestampedId { prevActiveSubmapIdx, timestamp };
		finishedSubmapsIdxs_.push(timestampedId);
		numScansMergedInActiveSubmap_ = 0;
		const auto id1 = submaps_.at(prevActiveSubmapIdx).getId();
		const auto id2 = submaps_.at(activeSubmapIdx_).getId();
		adjacencyMatrix_.addEdge(id1, id2);
//		std::cout << "Adding edge between " << id1 << " and " << id2 << std::endl;
		insertBufferedScans(&submaps_.at(activeSubmapIdx_));
		assert_true(!submaps_.at(activeSubmapIdx_).isEmpty(), "submap should not be empty after switching");
	} else {
		submaps_.at(activeSubmapIdx_).insertScan(rawScan, preProcessedScan, mapToRangeSensor, timestamp, true);
	}
	++numScansMergedInActiveSubmap_;
	return true;
}

void SubmapCollection::setParameters(const MapperParameters &p) {
	params_ = p;
	for (auto &submap : submaps_) {
		submap.setParameters(p);
	}
	placeRecognition_.setParameters(p);
	assert_gt<size_t>(params_.submaps_.numScansOverlap_, 0, "Num scan overlap has to be > 0");
	overlapScansBuffer_.set_size_limit(params_.submaps_.numScansOverlap_);
}

void SubmapCollection::computeFeatures(const TimestampedSubmapIds &finishedSubmapIds) {
	std::lock_guard<std::mutex> lck(featureComputationMutex_);
	isComputingFeatures_ = true;
	Timer timer("submap_finishing feature + constraint comp: ");

	auto featureComputation = [&]() {
//		Timer t("feature computation");
		for (const auto &id : finishedSubmapIds) {
//			std::cout << "computing features for submap: " << id.submapId_ << std::endl;
//			std::cout << "submap size: " << submaps_.at(id.submapId_).getMapPointCloud().points_.size() << std::endl;
			submaps_.at(id.submapId_).computeFeatures();
			loopClosureCandidatesIdxs_.push(id);
		}
	};

	std::thread t(featureComputation);

	{
//		Timer t("odometry_constraint_computation");
		computeOdometryConstraints(*this, finishedSubmapIds, &odometryConstraints_);
	}

	t.join();
	isComputingFeatures_ = false;
}

bool SubmapCollection::isComputingFeatures() const {
	return isComputingFeatures_;
}

const Constraints& SubmapCollection::getOdometryConstraints() const {
	return odometryConstraints_;
}

Constraints SubmapCollection::buildLoopClosureConstraints(
		const TimestampedSubmapIds &loopClosureCandidatesIdxs)  {
	Constraints retVal;
	for (const auto &id : loopClosureCandidatesIdxs) {
		const auto constraints = placeRecognition_.buildLoopClosureConstraints(mapToRangeSensor_, *this,
				adjacencyMatrix_, id.submapId_, activeSubmapIdx_, id.time_);
		if (!constraints.empty()) {
			for (int i = 0; i < constraints.size(); ++i) {
				retVal.push_back(constraints.at(i));
			}
			std::cout << " building loop closure constraints for submap: " << id.submapId_ << " resulted in: "
					<< constraints.size() << " new constraints \n";
		}
	}
	return retVal;
}

bool SubmapCollection::dumpToFile(const std::string &folderPath, const std::string &filename, const bool &isDenseMap) const {
	bool result = true;
	for (size_t i = 0; i < submaps_.size(); ++i) {
		PointCloud copy;
		if (isDenseMap) {
			copy = submaps_.at(i).getDenseMapCopy().toPointCloud();
		} else {
			copy = submaps_.at(i).getMapPointCloudCopy();
		}
		const std::string fullPath = folderPath + "/" + filename + "_" + std::to_string(i) + ".pcd";
		result = result && open3d::io::WritePointCloudToPCD(fullPath, copy, open3d::io::WritePointCloudOption());
	}
	return result;
}

void SubmapCollection::transform(const OptimizedTransforms &transformIncrements) {
	const size_t nTransforms = transformIncrements.size();
	std::vector<size_t> optimizedIdxs;
//	std::cout << "Num transforms: " << transformIncrements.size() << std::endl;
//	std::cout << "Num submaps: " << submaps_.size() << std::endl;
//	std::cout << "Transforms of updated submaps:: \n";
	for (size_t i = 0; i < nTransforms; ++i) {
		const auto &update = transformIncrements.at(i);
		if (update.submapId_ < submaps_.size()) {
			submaps_.at(update.submapId_).transform(update.dT_);
			optimizedIdxs.push_back(update.submapId_);
//			std::cout << "Submap " << update.submapId_ << " " << asString(update.dT_) << "\n";
		} else {
			std::cout << "tying to update submap: " << update.submapId_ << " but the there are only: "
					<< submaps_.size() << "submaps!!!! This should not happen! \n";
		}
	}
	std::sort(optimizedIdxs.begin(), optimizedIdxs.end());
	std::vector<size_t> allIdxs = getAllSubmapIdxs();

	std::vector<size_t> submapIdxsToUpdate;
	std::set_difference(allIdxs.begin(), allIdxs.end(), optimizedIdxs.begin(), optimizedIdxs.end(),
			std::inserter(submapIdxsToUpdate, submapIdxsToUpdate.begin()));
//	std::cout << "\n num maps: " << submaps_.size() << "\n";
//	std::cout << "num maps missing: " << submapIdxsToUpdate.size() << "\n";
//	std::cout << " maps that are missing: \n";
	for (auto idx : submapIdxsToUpdate) {
		//look at the node parent
		// if the parent is not in the list of nodes to update
		// then you are done, take the transform of the parent node
		// otherwise, set the current node to be the parent node
		size_t currentNode;
		currentNode = idx;
//		std::cout << currentNode << " with parent: ";
		while (true && !transformIncrements.empty()) {
			currentNode = submaps_.at(currentNode).getParentId();
			if (std::find(submapIdxsToUpdate.begin(), submapIdxsToUpdate.end(), currentNode)
					== submapIdxsToUpdate.end()) {
				/* parent is in the pose graph */
				const auto &update = transformIncrements.at(currentNode);
				submaps_.at(idx).transform(update.dT_);
//				std::cout << update.submapId_ << "\n";
				break;
			}
			if (currentNode == submaps_.at(currentNode).getParentId()) {
				throw std::runtime_error("Stuck in a loop, this should not happen");
			}
		}
	}

	//need to flush the buffered scans
	overlapScansBuffer_.clear();

}

std::vector<size_t> SubmapCollection::getAllSubmapIdxs() const {
	std::vector<size_t> idxs(submaps_.size());
	std::iota(idxs.begin(), idxs.end(), 0);
	return idxs;
}

const MapperParameters& SubmapCollection::getParameters() const {
	return params_;
}

void SubmapCollection::setFolderPath(const std::string &folderPath) {
	savingDataFolderPath_ = folderPath;
	placeRecognition_.setFolderPath(folderPath);
}

bool SubmapCollection::isSwitchingSubmapsConsistant(const PointCloud &scan,
		size_t newActiveSubmapCandidate, const Transform &mapToRangeSensor) const {
	//Timer("submap_switch_consistency_check");
	int numOverlappingPoints = 0;
	const VoxelMap &voxelMap = submaps_.at(newActiveSubmapCandidate).getVoxelMap();
	for (int i = 0; i < scan.points_.size(); ++i) {
		const Eigen::Vector3d p = mapToRangeSensor * scan.points_.at(i);
		numOverlappingPoints += int(voxelMap.hasVoxelContainingPoint(p));
	}
	const double fitness = static_cast<double>(numOverlappingPoints) / scan.points_.size();
//	std::cout << "Fitness: " << fitness << std::endl;
	return fitness > params_.submaps_.adjacencyBasedRevisitingMinFitness_;

}

} // namespace o3d_slam
