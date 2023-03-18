/*
 * SlamWrapper.hpp
 *
 *  Created on: Nov 23, 2021
 *      Author: jelavice
 */

#pragma once

#include <thread>
#include <future>
#include <Eigen/Dense>
#include "open3d_slam/Parameters.hpp"
#include "open3d_slam/Submap.hpp"
#include "open3d_slam/typedefs.hpp"
#include "open3d_slam/croppers.hpp"
#include "open3d_slam/TransformInterpolationBuffer.hpp"
#include "open3d_slam/CircularBuffer.hpp"
#include "open3d_slam/ThreadSafeBuffer.hpp"
#include "open3d_slam/Constraint.hpp"
#include <ros/ros.h>
#include <ros/package.h>
#include <chrono>


namespace o3d_slam {

class LidarOdometry;
class Mapper;
class SubmapCollection;
class OptimizationProblem;
class MotionCompensation;

class SlamWrapper {
	struct TimestampedPointCloud {
		Time time_;
		PointCloud cloud_;
	};


public:
	SlamWrapper();
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	virtual ~SlamWrapper();

	virtual void addRangeScan(const open3d::geometry::PointCloud cloud, const Time timestamp);
	virtual void loadParametersAndInitialize();
	virtual void startWorkers();
	virtual void stopWorkers();
	virtual void finishProcessing();

	struct RegisteredPointCloud{
		TimestampedPointCloud raw_;
		Transform transform_;
		std::string sourceFrame_, targetFrame_;
		size_t submapId_;
	};

	const MapperParameters &getMapperParameters() const;
	MapperParameters *getMapperParametersPtr();
	size_t getOdometryBufferSize() const;
	size_t getMappingBufferSize() const;
	size_t getOdometryBufferSizeLimit() const;
	size_t getMappingBufferSizeLimit() const;
	std::string getParameterFilePath() const;
	std::pair<PointCloud,Time> getLatestRegisteredCloudTimestampPair() const;

	void addOdometryPrior(const Time &queryTime, const Transform &transform);

	Transform getMapToRangeSensorPrior(const Time &timestamp) const;

	void setDirectoryPath(const std::string &path);
	void setMapSavingDirectoryPath(const std::string &path);
	void setParameterFilePath(const std::string &path);
	void setInitialMap(const PointCloud &initialMap);
	void setInitialTransform(const Eigen::Matrix4d initialTransform);

	bool saveMap(const std::string &directory);
	bool transformSaveMap(const std::string &directory, const Transform& transform);
	bool saveDenseSubmaps(const std::string &directory);
	bool saveSubmaps(const std::string &directory, const bool& isDenseMap=false);

	void setMapToEnu(const Transform& transform);
	Transform getMapToEnu();

	std::shared_ptr<LidarOdometry> odometry_;
	std::shared_ptr<Mapper> mapper_;

	bool checkIfodomToRangeSensorBufferHasTime(const Time &queryTime);

	Time latestScanToMapRefinementTimestamp_;
	Transform latestScanToMapRefinedPose_;
	bool isNewScan2MapRefinementAvailable_{false};

	Transform getLatestOdometryPose();
	SlamParameters params_;

	Transform mapToEnu_;

	RegisteredPointCloud getLatestRegisteredCloud();

	ros::Time getCurrentTimeAfterRegistration();
	ros::Time getCurrentTimeBeforeRegistration();

	std::chrono::high_resolution_clock::time_point getBeforeRegistrationTime();
	std::chrono::high_resolution_clock::time_point getAfterRegistrationTime();

	ros::Time currrentTimeAfterRegistration_;
	ros::Time currrentTimeBeforeRegistration_;

	std::chrono::high_resolution_clock::time_point beforeRegistration_;
	std::chrono::high_resolution_clock::time_point afterRegistration_;

	// Mutex
	mutable std::mutex posePublishingMutex_;


	bool hasMapPrior();

	bool hasOdometryPrior();

private:
	void checkIfOptimizedGraphAvailable();
	void odometryWorker();
	void mappingWorker();
	void loopClosureWorker();
	void computeFeaturesIfReady();
	void attemptLoopClosuresIfReady();
	void updateSubmapsAndTrajectory();
	void denseMapWorker();


protected:
	// buffers
	CircularBuffer<RegisteredPointCloud> registeredCloudBuffer_;
	CircularBuffer<TimestampedPointCloud> odometryBuffer_, mappingBuffer_;
	ThreadSafeBuffer<TimestampedSubmapId> loopClosureCandidates_;

	// parameters
	
//	MapperParameters mapperParams_;
//	OdometryParameters odometryParams_;
//	VisualizationParameters visualizationParameters_;
//	SavingParameters savingParameters_;
//	ConstantVelocityMotionCompensationParameters motionCompensationParameters_;
	std::string folderPath_, mapSavingFolderPath_;

	// modules
	std::shared_ptr<MotionCompensation> motionCompensationOdom_,motionCompensationMap_;
	std::shared_ptr<SubmapCollection> submaps_;
	std::shared_ptr<OptimizationProblem> optimizationProblem_;

	// multithreading
	std::thread odometryWorker_, mappingWorker_, loopClosureWorker_, denseMapWorker_;
	std::future<void> computeFeaturesResult_;

	// timing
	Timer mappingStatisticsTimer_,odometryStatisticsTimer_, visualizationUpdateTimer_, denseMapVisualizationUpdateTimer_, denseMapStatiscticsTimer_;
	Timer mapperOnlyTimer_;
	
	Time latestScanToScanRegistrationTimestamp_;

	// bookkeeping
	bool isOptimizedGraphAvailable_ = false;
	bool isRunWorkers_ = true;
	int numLatesLoopClosureConstraints_ = -1;
	PointCloud rawCloudPrev_;
	Constraints lastLoopClosureConstraints_;

};

} // namespace o3d_slam
