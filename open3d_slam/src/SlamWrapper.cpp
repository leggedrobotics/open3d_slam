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
#include "open3d_slam/OptimizationProblem.hpp"
#include "open3d_slam/constraint_builders.hpp"
#include "open3d_slam/Odometry.hpp"
#include "open3d_slam/MotionCompensation.hpp"
#include "open3d_slam/ScanToMapRegistration.hpp"

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
	if (params_.mapper_.isAttemptLoopClosures_ && loopClosureWorker_.joinable()) {
		loopClosureWorker_.join();
		std::cout << "Joined the loop closure worker \n";
	}

	if (params_.mapper_.isBuildDenseMap_ && denseMapWorker_.joinable()) {
		denseMapWorker_.join();
		std::cout << "Joined the dense map worker! \n";
	}

	std::cout << "    Scan insertion: Avg execution time: "
			<< mapperOnlyTimer_.getAvgMeasurementMsec() << " msec , frequency: "
			<< 1e3 / mapperOnlyTimer_.getAvgMeasurementMsec() << " Hz \n";

	if (params_.saving_.isSaveAtMissionEnd_){
		std::cout << "Saving maps .... \n";
		if (params_.saving_.isSaveMap_){
			saveMap(mapSavingFolderPath_);
		}
		if (params_.saving_.isSaveSubmaps_){
			saveSubmaps(mapSavingFolderPath_);
		}
		if (params_.mapper_.isBuildDenseMap_ && params_.saving_.isSaveDenseSubmaps_){
			saveDenseSubmaps(mapSavingFolderPath_);
		}
		std::cout << "All done! \n";
		std::cout << "Maps saved in " << mapSavingFolderPath_ << "\n";

	}
}

const MapperParameters &SlamWrapper::getMapperParameters() const{
	return params_.mapper_;
}

MapperParameters *SlamWrapper::getMapperParametersPtr(){
	return mapper_->getParametersPtr();
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
	numLatesLoopClosureConstraints_ = -1;
	submaps_->forceNewSubmapCreation();
	while (isRunWorkers_) {
		if (params_.mapper_.isAttemptLoopClosures_) {
			computeFeaturesIfReady();
			attemptLoopClosuresIfReady();
		} else {
			break;
		}
		if (numLatesLoopClosureConstraints_ == 0){
			break;
		}
		if (isOptimizedGraphAvailable_) {
			isOptimizedGraphAvailable_ = false;
			const auto poseBeforeUpdate = mapper_->getMapToRangeSensorBuffer().latest_measurement();
			std::cout << "latest pose before update: \n " << asStringXYZRPY(poseBeforeUpdate.transform_) << "\n";
			updateSubmapsAndTrajectory();
			const auto poseAfterUpdate = mapper_->getMapToRangeSensorBuffer().latest_measurement();
			std::cout << "latest pose after update: \n " << asStringXYZRPY(poseAfterUpdate.transform_) << "\n";
			if (params_.mapper_.isDumpSubmapsToFileBeforeAndAfterLoopClosures_) {
				submaps_->dumpToFile(folderPath_, "after", false);
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

void SlamWrapper::loadParametersAndInitialize() {

	//	auto &logger = open3d::utility::Logger::GetInstance();
	//	logger.SetVerbosityLevel(open3d::utility::VerbosityLevel::Debug);

	odometry_ = std::make_shared<o3d_slam::LidarOdometry>();
	odometry_->setParameters(params_.odometry_);

	submaps_ = std::make_shared<o3d_slam::SubmapCollection>();
	submaps_->setFolderPath(folderPath_);

	mapper_ = std::make_shared<o3d_slam::Mapper>(odometry_->getBuffer(), submaps_);
	mapper_->setParameters(params_.mapper_);

	optimizationProblem_ = std::make_shared<o3d_slam::OptimizationProblem>();
	optimizationProblem_->setParameters(params_.mapper_);

	// set the verobsity for timing statistics
	Timer::isDisablePrintInDestructor_ = !params_.mapper_.isPrintTimingStatistics_;

	if (params_.motionCompensation_.isUndistortInputCloud_){
		auto motionCompOdom = std::make_shared<ConstantVelocityMotionCompensation>(odometry_->getBuffer());
		motionCompOdom->setParameters(params_.motionCompensation_);
		motionCompensationOdom_ = motionCompOdom;
		auto motionCompMap = std::make_shared<ConstantVelocityMotionCompensation>(mapper_->getMapToRangeSensorBuffer());
		motionCompMap->setParameters(params_.motionCompensation_);
		motionCompensationMap_ = motionCompMap;
	}
}

void SlamWrapper::setInitialMap(const PointCloud &initialMap) {
	TimestampedPointCloud measurement{fromUniversal(0), std::move(initialMap)};
  {
	  Timer t("initial map preparation");
  	mapper_->getScanToMapRegistration().prepareInitialMap(&measurement.cloud_);
  }
  std::cout << "Initial map prepared! \n";
	const bool mappingResult = mapper_->addRangeMeasurement(measurement.cloud_, measurement.time_);
	if (!mappingResult) {
		std::cerr << "WARNING: mapping initialization has failed!!!! \n";
	}
}


void SlamWrapper::setInitialTransform(const Eigen::Matrix4d initialTransform) {
	odometry_->setInitialTransform(initialTransform);
	mapper_->setMapToRangeSensorInitial(Transform(initialTransform));
}

void SlamWrapper::startWorkers() {
	odometryWorker_ = std::thread([this]() {
		odometryWorker();
	});
	mappingWorker_ = std::thread([this]() {
		mappingWorker();
	});
	if (params_.mapper_.isAttemptLoopClosures_) {
		loopClosureWorker_ = std::thread([this]() {
			loopClosureWorker();
		});
	}
	if (params_.mapper_.isBuildDenseMap_) {
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
bool SlamWrapper::saveDenseSubmaps(const std::string &directory) {
	return saveSubmaps(directory, true);
}
bool SlamWrapper::saveSubmaps(const std::string &directory, const bool& isDenseMap) {
	createDirectoryOrNoActionIfExists(directory);
	const std::string cloudName = isDenseMap ? "denseSubmap" : "submap";
	const bool savingResult = mapper_->getSubmaps().dumpToFile(directory, cloudName, isDenseMap);
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
		if (params_.mapper_.isPrintTimingStatistics_ && odometryStatisticsTimer_.elapsedSec() > timingStatsEveryNsec) {
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
			checkIfOptimizedGraphAvailable();
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

		if (params_.mapper_.isAttemptLoopClosures_) {
			computeFeaturesIfReady();
			attemptLoopClosuresIfReady();
		}

		checkIfOptimizedGraphAvailable();

		//just get the stats
		const double timeMeasurement = mappingStatisticsTimer_.elapsedMsecSinceStopwatchStart();
		mappingStatisticsTimer_.addMeasurementMsec(timeMeasurement);
		if (params_.mapper_.isPrintTimingStatistics_ && mappingStatisticsTimer_.elapsedSec() > timingStatsEveryNsec) {
			std::cout << "Mapper timing stats: Avg execution time: "
					<< mappingStatisticsTimer_.getAvgMeasurementMsec() << " msec , frequency: "
					<< 1e3 / mappingStatisticsTimer_.getAvgMeasurementMsec() << " Hz \n";
			mappingStatisticsTimer_.reset();
		}

	} // while (isRunWorkers_)
}

void SlamWrapper::checkIfOptimizedGraphAvailable(){
	if (isOptimizedGraphAvailable_) {
		isOptimizedGraphAvailable_ = false;
		const auto poseBeforeUpdate = mapper_->getMapToRangeSensorBuffer().latest_measurement();
		std::cout << "latest pose before update: \n " << asStringXYZRPY(poseBeforeUpdate.transform_) << "\n";
		updateSubmapsAndTrajectory();
		const auto poseAfterUpdate = mapper_->getMapToRangeSensorBuffer().latest_measurement();
		std::cout << "latest pose after update: \n " << asStringXYZRPY(poseAfterUpdate.transform_) << "\n";
		if (params_.mapper_.isDumpSubmapsToFileBeforeAndAfterLoopClosures_){
			submaps_->dumpToFile(folderPath_, "after", false);
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
		if (params_.mapper_.isPrintTimingStatistics_ && denseMapStatiscticsTimer_.elapsedSec() > timingStatsEveryNsec) {
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
//			Timer t("loop_closing_attempt");
			const auto lcc = loopClosureCandidates_.popAllElements();
			loopClosureConstraints = submaps_->buildLoopClosureConstraints(lcc);
			numLatesLoopClosureConstraints_ = loopClosureConstraints.size();
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
			if (params_.mapper_.isDumpSubmapsToFileBeforeAndAfterLoopClosures_){
				submaps_->dumpToFile(folderPath_, "before", false);
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
			<< latestLoopClosureConstraint.sourceSubmapIdx_ << " the transform is: \n" << asStringXYZRPY(dT.dT_)
			<< std::endl;
	mapper_->loopClosureUpdate(dT.dT_);

	//now here you would update the lc constraints
	Constraints loopClosureConstraints = optimizationProblem_->getLoopClosureConstraints();
	for (int i = 0; i < loopClosureConstraints.size(); ++i) {
		const Constraint &oldConstraint = loopClosureConstraints.at(i);
		Constraint c = oldConstraint;
		c.sourceToTarget_.setIdentity();
		optimizationProblem_->updateLoopClosureConstraint(i, c);
		loopClosureConstraints.at(i) = c;
//		std::cout << "Loop closure constraint " << i << " new transform: " << asStringXYZRPY(c.sourceToTarget_)
//				<< std::endl;
	}

	submaps_->updateAdjacencyMatrix(loopClosureConstraints);

}







} // namespace o3d_slam

