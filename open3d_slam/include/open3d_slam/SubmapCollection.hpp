/*
 * SubmapCollection.hpp
 *
 *  Created on: Nov 10, 2021
 *      Author: jelavice
 */


#pragma once

#include <open3d/geometry/PointCloud.h>
#include <Eigen/Dense>
#include <mutex>
#include "open3d_slam/Parameters.hpp"
#include "open3d_slam/croppers.hpp"
#include "open3d_slam/Submap.hpp"
#include "open3d_slam/Constraint.hpp"
#include "open3d_slam/AdjacencyMatrix.hpp"
#include "open3d_slam/PlaceRecognition.hpp"
#include "open3d_slam/OptimizationProblem.hpp"
#include "open3d_slam/ThreadSafeBuffer.hpp"
#include "open3d_slam/CircularBuffer.hpp"


namespace o3d_slam {

class SubmapCollection {

	struct ScanTimeTransform{
		PointCloud cloud_;
		Time timestamp_;
		Transform mapToRangeSensor_;
	};

public:
	using SubmapId = Submap::SubmapId;
	using TimestampedSubmapIds = std::vector<TimestampedSubmapId>;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	SubmapCollection();
	~SubmapCollection() = default;

	void setMapToRangeSensor(const Transform &T);
	const Submap& getActiveSubmap() const;
	bool insertScan(const PointCloud &rawScan, const PointCloud &preProcessedScan,
			const Transform &mapToRangeSensor, const Time &timestamp);
	void setParameters(const MapperParameters &p);
	bool isEmpty() const;
	Submap* getSubmapPtr(SubmapId idx);
	const Submap &getSubmap(SubmapId idx) const;
	size_t getNumSubmaps() const;
	size_t getTotalNumPoints() const;

	void computeFeatures(const TimestampedSubmapIds &ids);
	bool isComputingFeatures() const;
	TimestampedSubmapIds popFinishedSubmapIds();
	size_t numFinishedSubmaps() const;


	Constraints buildLoopClosureConstraints(const TimestampedSubmapIds &ids);
	size_t numLoopClosureCandidates() const;
	TimestampedSubmapIds popLoopClosureCandidates();


	bool dumpToFile(const std::string &folderPath, const std::string &filename, const bool& isDenseMap) const;
	void transform(const OptimizedTransforms &transformIncrements);
	void updateAdjacencyMatrix(const Constraints &loopClosureConstraints);
	const Constraints &getOdometryConstraints() const;

	const MapperParameters &getParameters() const;
	void setFolderPath(const std::string &folderPath);

	void forceNewSubmapCreation();

private:
	bool isSwitchingSubmapsConsistant(const PointCloud &scan, size_t newActiveSubmapCandidate, const Transform &mapToRangeSensor) const;
	void insertBufferedScans(Submap *submap);
	void addScanToBuffer(const PointCloud &scan, const Transform &mapToRangeSensor, const Time &timestamp);
	void updateActiveSubmap(const Transform &mapToRangeSensor, const PointCloud &scan);
	void createNewSubmap(const Transform &mapToSubmap);
	size_t findClosestSubmap(const Transform &mapToRangesensor) const;
	std::vector<size_t> getAllSubmapIdxs() const;

	Transform mapToRangeSensor_ = Transform::Identity();
	Time timestamp_;
	std::vector<Submap> submaps_;
	size_t activeSubmapIdx_ = 0;
	MapperParameters params_;
	size_t numScansMergedInActiveSubmap_ = 0;
	size_t lastFinishedSubmapIdx_ = 0;
	std::mutex featureComputationMutex_;
	bool isComputingFeatures_ = false;
	std::mutex constraintBuildMutex_;
	AdjacencyMatrix adjacencyMatrix_;
	size_t submapId_=0;
	PlaceRecognition placeRecognition_;
	ThreadSafeBuffer<TimestampedSubmapId> loopClosureCandidatesIdxs_, finishedSubmapsIdxs_;
	Constraints odometryConstraints_;
	CircularBuffer<ScanTimeTransform> overlapScansBuffer_;
	std::string savingDataFolderPath_;
	bool isForceNewSubmapCreation_ = false;
};

} // namespace o3d_slam
