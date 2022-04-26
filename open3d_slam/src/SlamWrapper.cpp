/*
 * SlamWrapper.cpp
 *
 *  Created on: Nov 23, 2021
 *      Author: jelavice
 */

#include "open3d_slam/SlamWrapper.hpp"

#include <chrono>
#include <open3d/Open3D.h>
#include "open3d_slam/Parameters.hpp"
#include "open3d_slam/frames.hpp"
#include "open3d_slam/helpers.hpp"
#include "open3d_slam/time.hpp"
#include "open3d_slam/math.hpp"
#include "open3d_slam/croppers.hpp"
#include "open3d_slam/output.hpp"
#include "open3d_slam/Mapper.hpp"
#include "open3d_slam/assert.hpp"
#include "open3d_slam/Mesher.hpp"
#include "open3d_slam/OptimizationProblem.hpp"
#include "open3d_slam/constraint_builders.hpp"
#include "open3d_slam/Odometry.hpp"
#include "open3d_slam/MotionCompensation.hpp"

#ifdef open3d_slam_OPENMP_FOUND
#include <omp.h>
#endif

namespace o3d_slam {

namespace {
using namespace o3d_slam::frames;
const double timingStatsEveryNsec = 15.0;
}

SlamWrapper::SlamWrapper() {
	//todo magic
	odometryBuffer_.set_size_limit(30);
	mappingBuffer_.set_size_limit(30);
	registeredCloudBuffer_.set_size_limit(30);
	motionCompensationOdom_ = std::make_shared<MotionCompensation>();
	motionCompensationMap_ = std::make_shared<MotionCompensation>();
}

SlamWrapper::~SlamWrapper() {
	if (odometryWorker_.joinable()) {
		odometryWorker_.join();
		std::cout << "Joined odometry worker \n";
	}
	if (mappingWorker_.joinable()) {
		mappingWorker_.join();
		std::cout << "Joined mapping worker \n";
	}
	if (mapperParams_.isAttemptLoopClosures_ && loopClosureWorker_.joinable()) {
		loopClosureWorker_.join();
		std::cout << "Joined the loop closure worker \n";
	}

	if (mapperParams_.isBuildDenseMap_ && denseMapWorker_.joinable()) {
		denseMapWorker_.join();
		std::cout << "Joined the dense map worker! \n";
	}

	std::cout << "    Scan insertion: Avg execution time: "
			<< mapperOnlyTimer_.getAvgMeasurementMsec() << " msec , frequency: "
			<< 1e3 / mapperOnlyTimer_.getAvgMeasurementMsec() << " Hz \n";

	if (savingParameters_.isSaveAtMissionEnd_){
		std::cout << "Saving maps .... \n";
		if (savingParameters_.isSaveMap_){
			saveMap(mapSavingFolderPath_);
		}
		if (savingParameters_.isSaveSubmaps_){
			saveSubmaps(mapSavingFolderPath_);
		}
		std::cout << "All done! \n";
		std::cout << "Maps saved in " << mapSavingFolderPath_ << "\n";

	}
}

size_t SlamWrapper::getOdometryBufferSize() const {
	return odometryBuffer_.size();
}
size_t SlamWrapper::getMappingBufferSize() const {
	return mappingBuffer_.size();
}

size_t SlamWrapper::getOdometryBufferSizeLimit() const {
	return odometryBuffer_.size_limit();
}
size_t SlamWrapper::getMappingBufferSizeLimit() const {
	return mappingBuffer_.size_limit();
}

void SlamWrapper::addRangeScan(const open3d::geometry::PointCloud cloud, const Time timestamp) {
	updateFirstMeasurementTime(timestamp);

	auto removedNans = removePointsWithNonFiniteValues(cloud);
	const TimestampedPointCloud timestampedCloud { timestamp, *removedNans };
	if (!odometryBuffer_.empty()) {
		const auto latestTime = odometryBuffer_.peek_back().time_;
		if (timestamp < latestTime) {
			std::cerr << "you are trying to add a range scan out of order! Dropping the measurement! \n";
			return;
		}
	}
	odometryBuffer_.push(timestampedCloud);
}

std::pair<PointCloud, Time> SlamWrapper::getLatestRegisteredCloudTimestampPair() const {
	if (registeredCloudBuffer_.empty()) {
		return {PointCloud(),Time()};
	}
	RegisteredPointCloud c = registeredCloudBuffer_.peek_back();
//	c.raw_.cloud_.Transform(c.transform_.matrix());
	return {c.raw_.cloud_,c.raw_.time_};
}

void SlamWrapper::finishProcessing() {
	while (isRunWorkers_) {
		if (!mappingBuffer_.empty()) {
			std::cout << "  Waiting for the mapping buffer to be emptied \n";
			std::this_thread::sleep_for(std::chrono::milliseconds(200));
			continue;
		} else {
			std::cout << "  Mapping buffer emptied \n";
			break;
		}
	}
	std::cout << "Finishing all submaps! \n";
	submaps_->forceNewSubmapCreation();
	while (isRunWorkers_) {
		if (mapperParams_.isAttemptLoopClosures_) {
			computeFeaturesIfReady();
			attemptLoopClosuresIfReady();
		}

		if (isOptimizedGraphAvailable_) {
			isOptimizedGraphAvailable_ = false;
			const auto poseBeforeUpdate = mapper_->getMapToRangeSensorBuffer().latest_measurement();
			std::cout << "latest pose before update: \n " << asString(poseBeforeUpdate.transform_) << "\n";
			updateSubmapsAndTrajectory();
			const auto poseAfterUpdate = mapper_->getMapToRangeSensorBuffer().latest_measurement();
			std::cout << "latest pose after update: \n " << asString(poseAfterUpdate.transform_) << "\n";
			if (mapperParams_.isDumpSubmapsToFileBeforeAndAfterLoopClosures_) {
				submaps_->dumpToFile(folderPath_, "after");
			}
			break;
		} else {
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
	}
	std::cout << "All submaps fnished! \n";
}

void SlamWrapper::setDirectoryPath(const std::string &path){
	folderPath_ = path;
}
void SlamWrapper::setMapSavingDirectoryPath(const std::string &path){
	mapSavingFolderPath_ = path;
}

void SlamWrapper::setParameterFilePath(const std::string &path){
	paramPath_ = path;
}

void SlamWrapper::loadParametersAndInitialize() {

	//	auto &logger = open3d::utility::Logger::GetInstance();
	//	logger.SetVerbosityLevel(open3d::utility::VerbosityLevel::Debug);

	const std::string paramFile = paramPath_;
	std::cout << "loading params from: " << paramFile << "\n";

	o3d_slam::loadParameters(paramFile, &localMapParams_);

	OdometryParameters odometryParams;
	loadParameters(paramFile, &odometryParams);
	odometry_ = std::make_shared<o3d_slam::LidarOdometry>();
	odometry_->setParameters(odometryParams);

	//todo remove magic
	const double lidarFrequency = 10.0;
	int numMeasurementsVelocityEstimation = 3;
	auto motionComp = std::make_shared<ConstantVelocityMotionCompensation>(odometry_->getBuffer());
	motionComp->setParameters(lidarFrequency, numMeasurementsVelocityEstimation);
//	motionCompensationOdom_ = motionComp;


	submaps_ = std::make_shared<o3d_slam::SubmapCollection>();
	submaps_->setFolderPath(folderPath_);
	mapper_ = std::make_shared<o3d_slam::Mapper>(odometry_->getBuffer(), submaps_);
	o3d_slam::loadParameters(paramFile, &mapperParams_);
	mapper_->setParameters(mapperParams_);

	motionComp = std::make_shared<ConstantVelocityMotionCompensation>(mapper_->getMapToRangeSensorBuffer());
	motionComp->setParameters(lidarFrequency, numMeasurementsVelocityEstimation);
//	motionCompensationMap_ = motionComp;

	optimizationProblem_ = std::make_shared<o3d_slam::OptimizationProblem>();
	optimizationProblem_->setParameters(mapperParams_);

	loadParameters(paramFile, &visualizationParameters_);

	// set the verobsity for timing statistics
	Timer::isDisablePrintInDestructor_ = !mapperParams_.isPrintTimingStatistics_;

	loadParameters(paramFile,&savingParameters_);

}

void SlamWrapper::startWorkers() {
	odometryWorker_ = std::thread([this]() {
		odometryWorker();
	});
	mappingWorker_ = std::thread([this]() {
		mappingWorker();
	});
	if (mapperParams_.isAttemptLoopClosures_) {
		loopClosureWorker_ = std::thread([this]() {
			loopClosureWorker();
		});
	}
	if (mapperParams_.isBuildDenseMap_) {
		denseMapWorker_ = std::thread([this]() {
			denseMapWorker();
		});
	}

}

void SlamWrapper::stopWorkers(){
	isRunWorkers_ = false;
}

bool SlamWrapper::saveMap(const std::string &directory) {
	PointCloud map = mapper_->getAssembledMapPointCloud();
	createDirectoryOrNoActionIfExists(directory);
	const std::string filename = directory + "map.pcd";
	return saveToFile(filename, map);
}
bool SlamWrapper::saveSubmaps(const std::string &directory) {
	createDirectoryOrNoActionIfExists(directory);
	const bool savingResult = mapper_->getSubmaps().dumpToFile(directory, "submap");
	return savingResult;
}

void SlamWrapper::odometryWorker() {
	while (isRunWorkers_) {
		if (odometryBuffer_.empty()) {
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
			continue;
		}
		odometryStatisticsTimer_.startStopwatch();
		const TimestampedPointCloud measurement = odometryBuffer_.pop();
		auto undistortedCloud = motionCompensationOdom_->undistortInputPointCloud(measurement.cloud_, measurement.time_);

		const auto isOdomOkay = odometry_->addRangeScan(*undistortedCloud, measurement.time_);

		// this ensures that the odom is always ahead of the mapping
		// so then we can look stuff up in the interpolation buffer
		mappingBuffer_.push(measurement);

		if (!isOdomOkay) {
			std::cerr << "WARNING: odometry has failed!!!! \n";
			continue;
		}

		latestScanToScanRegistrationTimestamp_ = measurement.time_;

		const double timeMeasurement = odometryStatisticsTimer_.elapsedMsecSinceStopwatchStart();
		odometryStatisticsTimer_.addMeasurementMsec(timeMeasurement);
		if (mapperParams_.isPrintTimingStatistics_ && odometryStatisticsTimer_.elapsedSec() > timingStatsEveryNsec) {
			std::cout << "Odometry timing stats: Avg execution time: "
					<< odometryStatisticsTimer_.getAvgMeasurementMsec() << " msec , frequency: "
					<< 1e3 / odometryStatisticsTimer_.getAvgMeasurementMsec() << " Hz \n";
			odometryStatisticsTimer_.reset();
		}

	} // end while

}
void SlamWrapper::mappingWorker() {
	while (isRunWorkers_) {
		if (mappingBuffer_.empty()) {
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
			continue;
		}
		mappingStatisticsTimer_.startStopwatch();
		TimestampedPointCloud measurement;
		{
			const TimestampedPointCloud raw = mappingBuffer_.pop();
			auto undistortedCloud =
					motionCompensationMap_->undistortInputPointCloud(raw.cloud_,
							raw.time_);
			measurement.time_ = raw.time_;
			measurement.cloud_ = *undistortedCloud;
		}
		if (!odometry_->getBuffer().has(measurement.time_)) {
			std::cout << "Weird, the odom buffer does not seem to have the transform!!! \n";
			std::cout << "odom buffer size: " << odometry_->getBuffer().size() << "/"
					<< odometry_->getBuffer().size_limit() << std::endl;
			const auto &b = odometry_->getBuffer();
			std::cout << "earliest: " << toSecondsSinceFirstMeasurement(b.earliest_time()) << std::endl;
			std::cout << "latest: " << toSecondsSinceFirstMeasurement(b.latest_time()) << std::endl;
			std::cout << "requested: " << toSecondsSinceFirstMeasurement(measurement.time_) << std::endl;
		}
		const size_t activeSubmapIdx = mapper_->getActiveSubmap().getId();
		mapperOnlyTimer_.startStopwatch();
		const bool mappingResult = mapper_->addRangeMeasurement(measurement.cloud_, measurement.time_);
		const double timeElapsed = 	mapperOnlyTimer_.elapsedMsecSinceStopwatchStart();
		mapperOnlyTimer_.addMeasurementMsec(timeElapsed);

		if (mappingResult) {
			RegisteredPointCloud registeredCloud;
			registeredCloud.submapId_ = activeSubmapIdx;
			registeredCloud.raw_ = measurement;
			registeredCloud.transform_ = mapper_->getMapToRangeSensor(measurement.time_);
			registeredCloud.sourceFrame_ = frames::rangeSensorFrame;
			registeredCloud.targetFrame_ = frames::mapFrame;
			registeredCloudBuffer_.push(registeredCloud);
			latestScanToMapRefinementTimestamp_ = measurement.time_;
		}

		if (mapperParams_.isAttemptLoopClosures_) {
			computeFeaturesIfReady();
			attemptLoopClosuresIfReady();
		}

		if (isOptimizedGraphAvailable_) {
			isOptimizedGraphAvailable_ = false;
			const auto poseBeforeUpdate = mapper_->getMapToRangeSensorBuffer().latest_measurement();
			std::cout << "latest pose before update: \n " << asString(poseBeforeUpdate.transform_) << "\n";
			updateSubmapsAndTrajectory();
			const auto poseAfterUpdate = mapper_->getMapToRangeSensorBuffer().latest_measurement();
			std::cout << "latest pose after update: \n " << asString(poseAfterUpdate.transform_) << "\n";
//			publishMaps(measurement.time_);
			if (mapperParams_.isDumpSubmapsToFileBeforeAndAfterLoopClosures_){
				submaps_->dumpToFile(folderPath_, "after");
			}
		}

		// publish visualizatinos
//		publishMaps(measurement.time_);

		//just get the stats
		const double timeMeasurement = mappingStatisticsTimer_.elapsedMsecSinceStopwatchStart();
		mappingStatisticsTimer_.addMeasurementMsec(timeMeasurement);
		if (mapperParams_.isPrintTimingStatistics_ && mappingStatisticsTimer_.elapsedSec() > timingStatsEveryNsec) {
			std::cout << "Mapper timing stats: Avg execution time: "
					<< mappingStatisticsTimer_.getAvgMeasurementMsec() << " msec , frequency: "
					<< 1e3 / mappingStatisticsTimer_.getAvgMeasurementMsec() << " Hz \n";
			mappingStatisticsTimer_.reset();
		}

	}
}

void SlamWrapper::denseMapWorker() {
	while (isRunWorkers_) {
		if (registeredCloudBuffer_.empty()) {
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
			continue;
		}
		denseMapStatiscticsTimer_.startStopwatch();

		const RegisteredPointCloud regCloud = registeredCloudBuffer_.pop();

		mapper_->getSubmapsPtr()->getSubmapPtr(regCloud.submapId_)->insertScanDenseMap(regCloud.raw_.cloud_,
					regCloud.transform_, regCloud.raw_.time_, true);

		const double timeMeasurement = denseMapStatiscticsTimer_.elapsedMsecSinceStopwatchStart();
		denseMapStatiscticsTimer_.addMeasurementMsec(timeMeasurement);
		if (mapperParams_.isPrintTimingStatistics_ && denseMapStatiscticsTimer_.elapsedSec() > timingStatsEveryNsec) {
			std::cout << "Dense mapping timing stats: Avg execution time: "
					<< denseMapStatiscticsTimer_.getAvgMeasurementMsec() << " msec , frequency: "
					<< 1e3 / denseMapStatiscticsTimer_.getAvgMeasurementMsec() << " Hz \n";
			denseMapStatiscticsTimer_.reset();
		}

	} // end while

}

void SlamWrapper::computeFeaturesIfReady() {
	if (submaps_->numFinishedSubmaps() > 0 && !submaps_->isComputingFeatures()) {
		computeFeaturesResult_ = std::async(std::launch::async, [this]() {
			const auto finishedSubmapIds = submaps_->popFinishedSubmapIds();
			submaps_->computeFeatures(finishedSubmapIds);
		});
	}
}
void SlamWrapper::attemptLoopClosuresIfReady() {
	const auto timeout = std::chrono::milliseconds(0);
	if (computeFeaturesResult_.valid()
			&& computeFeaturesResult_.wait_for(timeout) == std::future_status::ready) {
		computeFeaturesResult_.get(); // consume the future
		if (submaps_->numLoopClosureCandidates() > 0) {
			const auto lcc = submaps_->popLoopClosureCandidates();
			loopClosureCandidates_.insert(lcc.begin(), lcc.end());
		}
	}
}
void SlamWrapper::loopClosureWorker() {
	while (isRunWorkers_) {
		if (loopClosureCandidates_.empty() || isOptimizedGraphAvailable_) {
			std::this_thread::sleep_for(std::chrono::milliseconds(200));
			continue;
		}

		Constraints loopClosureConstraints;
		{
			Timer t("loop_closing_attempt");
			const auto lcc = loopClosureCandidates_.popAllElements();
			loopClosureConstraints = submaps_->buildLoopClosureConstraints(lcc);
		}

		if (loopClosureConstraints.empty()) {
			std::this_thread::sleep_for(std::chrono::milliseconds(200));
			continue;
		}
		{
			Timer t("optimization_problem");
			auto odometryConstraints = submaps_->getOdometryConstraints();
			computeOdometryConstraints(*submaps_, &odometryConstraints);

//			optimizationProblem_->clearLoopClosureConstraints();
			optimizationProblem_->clearOdometryConstraints();
			optimizationProblem_->insertLoopClosureConstraints(loopClosureConstraints);
			optimizationProblem_->insertOdometryConstraints(odometryConstraints);
			optimizationProblem_->buildOptimizationProblem(*submaps_);

//			optimizationProblem_->print();
			if (mapperParams_.isDumpSubmapsToFileBeforeAndAfterLoopClosures_){
				submaps_->dumpToFile(folderPath_, "before");
				optimizationProblem_->dumpToFile(folderPath_ + "/poseGraph.json");
			}
			optimizationProblem_->solve();
			//optimizationProblem_->print();
			lastLoopClosureConstraints_ = loopClosureConstraints;
			isOptimizedGraphAvailable_ = true;
		}

	} // end while

}

void SlamWrapper::updateSubmapsAndTrajectory() {

	std::cout << "Updating the maps: \n";
	const Timer t("submaps_update");
	const auto optimizedTransformations = optimizationProblem_->getOptimizedTransformIncrements();
	//todo this segfault!!!!
	submaps_->transform(optimizedTransformations);

	//get The correct time
	const Constraint latestLoopClosureConstraint = *std::max_element(lastLoopClosureConstraints_.begin(),
			lastLoopClosureConstraints_.end(), [](const Constraint &c1, const Constraint &c2) {
				return c1.timestamp_ < c2.timestamp_;
			});

	// loop closing constraints are built such that the source node is always the ne being transformed
	// hence the source node should be the one whose transform we should apply
	assert_gt(latestLoopClosureConstraint.sourceSubmapIdx_, latestLoopClosureConstraint.targetSubmapIdx_,
			"Wrapper ros, update submaps and trajectory: ");
	const auto dT = optimizedTransformations.at(latestLoopClosureConstraint.sourceSubmapIdx_);

	std::cout << "Transforming the pose buffer with the delta T from submap "
			<< latestLoopClosureConstraint.sourceSubmapIdx_ << "the transform is: \n" << asString(dT.dT_)
			<< std::endl;
	mapper_->loopClosureUpdate(dT.dT_);

	//now here you would update the lc constraints
	Constraints loopClosureConstraints = optimizationProblem_->getLoopClosureConstraints();
//#pragma omp parallel for
	for (int i = 0; i < loopClosureConstraints.size(); ++i) {
		const Constraint &oldConstraint = loopClosureConstraints.at(i);
		Constraint c = oldConstraint;
		c.sourceToTarget_.setIdentity();
		optimizationProblem_->updateLoopClosureConstraint(i, c);
		loopClosureConstraints.at(i) = c;
		std::cout << "Loop closure constraint " << i << " new transform: " << asString(c.sourceToTarget_)
				<< std::endl;
	}

	submaps_->updateAdjacencyMatrix(loopClosureConstraints);

}







} // namespace o3d_slam

