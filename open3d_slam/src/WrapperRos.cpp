/*
 * WrapperRos.cpp
 *
 *  Created on: Nov 23, 2021
 *      Author: jelavice
 */

#include "open3d_slam/WrapperRos.hpp"

#include <chrono>
#include <open3d/Open3D.h>
#include "open3d_slam/Parameters.hpp"
#include "open3d_slam/frames.hpp"
#include "open3d_slam/helpers.hpp"
#include "open3d_slam/helpers_ros.hpp"
#include "open3d_slam/time.hpp"
#include "open3d_slam/math.hpp"
#include "open3d_slam/croppers.hpp"
#include "open3d_slam/Mapper.hpp"
#include "open3d_slam/assert.hpp"
#include "open3d_slam/Mesher.hpp"
#include "open3d_slam/OptimizationProblem.hpp"

#include "open3d_slam/Odometry.hpp"
#include "open3d_conversions/open3d_conversions.h"
#include <open3d_slam_msgs/PolygonMesh.h>

#ifdef open3d_slam_OPENMP_FOUND
#include <omp.h>
#endif

namespace o3d_slam {

namespace {
using namespace o3d_slam::frames;
const double timingStatsEveryNsec = 15.0;
}

WrapperRos::WrapperRos(ros::NodeHandlePtr nh) :
		nh_(nh) {
	tfBroadcaster_.reset(new tf2_ros::TransformBroadcaster());
	//todo magic
	odometryBuffer_.set_size_limit(30);
	mappingBuffer_.set_size_limit(40);
	registeredCloudBuffer_.set_size_limit(50);

}

WrapperRos::~WrapperRos() {
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
}

size_t WrapperRos::getOdometryBufferSize() const {
	return odometryBuffer_.size();
}
size_t WrapperRos::getMappingBufferSize() const {
	return mappingBuffer_.size();
}

size_t WrapperRos::getOdometryBufferSizeLimit() const {
	return odometryBuffer_.size_limit();
}
size_t WrapperRos::getMappingBufferSizeLimit() const {
	return mappingBuffer_.size_limit();
}

void WrapperRos::addRangeScan(const open3d::geometry::PointCloud cloud, const Time timestamp) {
	const TimestampedPointCloud timestampedCloud { timestamp, cloud };
	if (!odometryBuffer_.empty()) {
		const auto latestTime = odometryBuffer_.peek_back().time_;
		if (timestamp < latestTime) {
			std::cerr << "you are trying to add a range scan out of order! Dropping the measurement! \n";
			return;
		}
	}
	odometryBuffer_.push(timestampedCloud);
}

std::pair<PointCloud, Time> WrapperRos::getLatestRegisteredCloudTimestampPair() const {
	if (registeredCloudBuffer_.empty()) {
		return {PointCloud(),Time()};
	}
	RegisteredPointCloud c = registeredCloudBuffer_.peek_back();
//	c.raw_.cloud_.Transform(c.transform_.matrix());
	return {c.raw_.cloud_,c.raw_.time_};
}

void WrapperRos::initialize() {

	odometryInputPub_ = nh_->advertise<sensor_msgs::PointCloud2>("odom_input", 1, true);
	mappingInputPub_ = nh_->advertise<sensor_msgs::PointCloud2>("mapping_input", 1, true);
	assembledMapPub_ = nh_->advertise<sensor_msgs::PointCloud2>("assembled_map", 1, true);
	denseMapPub_ = nh_->advertise<sensor_msgs::PointCloud2>("dense_map", 1, true);
	meshPub_ = nh_->advertise<open3d_slam_msgs::PolygonMesh>("mesh", 1, true);

	submapsPub_ = nh_->advertise<sensor_msgs::PointCloud2>("submaps", 1, true);
	submapOriginsPub_ = nh_->advertise<visualization_msgs::MarkerArray>("submap_origins", 1, true);

	//	auto &logger = open3d::utility::Logger::GetInstance();
	//	logger.SetVerbosityLevel(open3d::utility::VerbosityLevel::Debug);

	folderPath_ = ros::package::getPath("open3d_slam") + "/data/";

	const std::string paramFile = nh_->param<std::string>("parameter_file_path", "");
	std::cout << "loading params from: " << paramFile << "\n";

	o3d_slam::loadParameters(paramFile, &localMapParams_);

	OdometryParameters odometryParams;
	loadParameters(paramFile, &odometryParams);
	odometry_ = std::make_shared<o3d_slam::LidarOdometry>();
	odometry_->setParameters(odometryParams);

	submaps_ = std::make_shared<o3d_slam::SubmapCollection>();
	submaps_->setFolderPath(folderPath_);
	mapper_ = std::make_shared<o3d_slam::Mapper>(odometry_->getBuffer(), submaps_);
	o3d_slam::loadParameters(paramFile, &mapperParams_);
	mapper_->setParameters(mapperParams_);

	optimizationProblem_ = std::make_shared<o3d_slam::OptimizationProblem>();
	optimizationProblem_->setParameters(mapperParams_);

	mesher_ = std::make_shared<o3d_slam::Mesher>();
	o3d_slam::loadParameters(paramFile, &mesherParams_);
	mesher_->setParameters(mesherParams_);

	loadParameters(paramFile, &visualizationParameters_);

}

void WrapperRos::start() {
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

void WrapperRos::odometryWorker() {
	while (ros::ok()) {
		if (odometryBuffer_.empty()) {
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
			continue;
		}
		odometryStatisticsTimer_.startStopwatch();
		const TimestampedPointCloud measurement = odometryBuffer_.pop();
		if (!odometry_->addRangeScan(measurement.cloud_, measurement.time_)) {
			std::cerr << "WARNING: odometry has failed!!!! \n";
			continue;
		}
		// this ensures that the odom is always ahead of the mapping
		// so then we can look stuff up in the interpolation buffer
		mappingBuffer_.push(measurement);
		const auto timestamp = toRos(measurement.time_);
		o3d_slam::publishTfTransform(odometry_->getOdomToRangeSensor(measurement.time_).matrix(), timestamp,
				odomFrame, rangeSensorFrame, tfBroadcaster_.get());
		o3d_slam::publishTfTransform(odometry_->getOdomToRangeSensor(measurement.time_).matrix(), timestamp,
				mapFrame, "raw_odom_o3d", tfBroadcaster_.get());

		if (odometryInputPub_.getNumSubscribers() > 0) {
			auto odomInput = odometry_->getPreProcessedCloud();
			std::thread t([this, timestamp, odomInput]() {
				o3d_slam::publishCloud(odomInput, o3d_slam::frames::rangeSensorFrame, timestamp, odometryInputPub_);
			});
			t.detach();
		}
		const double timeMeasurement = odometryStatisticsTimer_.elapsedMsecSinceStopwatchStart();
		odometryStatisticsTimer_.addMeasurementMsec(timeMeasurement);
		if (odometryStatisticsTimer_.elapsedSec() > timingStatsEveryNsec) {
			std::cout << "Odometry timing stats: Avg execution time: "
					<< odometryStatisticsTimer_.getAvgMeasurementMsec() << " msec , frequency: "
					<< 1e3 / odometryStatisticsTimer_.getAvgMeasurementMsec() << " Hz \n";
			odometryStatisticsTimer_.reset();
		}

	} // end while

}
void WrapperRos::mappingWorker() {
	while (ros::ok()) {
		if (mappingBuffer_.empty()) {
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
			continue;
		}
		mappingStatisticsTimer_.startStopwatch();
		const TimestampedPointCloud measurement = mappingBuffer_.pop();

		if (!odometry_->getBuffer().has(measurement.time_)) {
			std::cout << "Weird, the odom buffer does not seem to have the transform!!! \n";
			std::cout << "odom buffer size: " << odometry_->getBuffer().size() << "/"
					<< odometry_->getBuffer().size_limit() << std::endl;
			const auto &b = odometry_->getBuffer();
			std::cout << "earliest: " << readable(b.earliest_time()) << std::endl;
			std::cout << "latest: " << readable(b.latest_time()) << std::endl;
			std::cout << "requested: " << readable(measurement.time_) << std::endl;
		}
		const size_t activeSubmapIdx = mapper_->getActiveSubmap().getId();
		const bool mappingResult = mapper_->addRangeMeasurement(measurement.cloud_, measurement.time_);
		publishMapToOdomTf(measurement.time_);
		//mesherBufffer_.push(measurement.time_);

		if (mappingResult) {
			RegisteredPointCloud registeredCloud;
			registeredCloud.submapId_ = activeSubmapIdx;
			registeredCloud.raw_ = measurement;
			registeredCloud.transform_ = mapper_->getMapToRangeSensor(measurement.time_);
			registeredCloud.sourceFrame_ = frames::rangeSensorFrame;
			registeredCloud.targetFrame_ = frames::mapFrame;
			registeredCloudBuffer_.push(registeredCloud);
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
			publishMaps(measurement.time_);
			if (mapperParams_.isDumpSubmapsToFileBeforeAndAfterLoopClosures_){
				submaps_->dumpToFile(folderPath_, "after");
			}
		}

		// publish visualizatinos
		publishMaps(measurement.time_);

		//just get the stats
		const double timeMeasurement = mappingStatisticsTimer_.elapsedMsecSinceStopwatchStart();
		mappingStatisticsTimer_.addMeasurementMsec(timeMeasurement);
		if (mappingStatisticsTimer_.elapsedSec() > timingStatsEveryNsec) {
			std::cout << "Mapper timing stats: Avg execution time: "
					<< mappingStatisticsTimer_.getAvgMeasurementMsec() << " msec , frequency: "
					<< 1e3 / mappingStatisticsTimer_.getAvgMeasurementMsec() << " Hz \n";
			mappingStatisticsTimer_.reset();
		}

	}
}

void WrapperRos::denseMapWorker() {
	while (ros::ok()) {
		if (registeredCloudBuffer_.empty()) {
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
			continue;
		}
		denseMapStatiscticsTimer_.startStopwatch();
		const RegisteredPointCloud regCloud = registeredCloudBuffer_.pop();

		mapper_->getSubmapsPtr()->getSubmapPtr(regCloud.submapId_)->insertScanDenseMap(regCloud.raw_.cloud_,
				regCloud.transform_, regCloud.raw_.time_, true);

		if (mapper_->getDenseMap().HasColors()
				&& denseMapVisualizationUpdateTimer_.elapsedMsec() > visualizationParameters_.visualizeEveryNmsec_) {
			const auto denseMap = mapper_->getDenseMap(); //copy
			const auto timestamp = toRos(regCloud.raw_.time_);
			o3d_slam::publishCloud(denseMap, o3d_slam::frames::mapFrame, timestamp, denseMapPub_);
			denseMapVisualizationUpdateTimer_.reset();
		}

		const double timeMeasurement = denseMapStatiscticsTimer_.elapsedMsecSinceStopwatchStart();
		denseMapStatiscticsTimer_.addMeasurementMsec(timeMeasurement);
		if (denseMapStatiscticsTimer_.elapsedSec() > timingStatsEveryNsec) {
			std::cout << "Dense mapping timing stats: Avg execution time: "
					<< denseMapStatiscticsTimer_.getAvgMeasurementMsec() << " msec , frequency: "
					<< 1e3 / denseMapStatiscticsTimer_.getAvgMeasurementMsec() << " Hz \n";
			denseMapStatiscticsTimer_.reset();
		}

	} // end while

}

void WrapperRos::publishMapToOdomTf(const Time &time) {
	const auto timestamp = toRos(time);
	o3d_slam::publishTfTransform(mapper_->getMapToOdom(time).matrix(), timestamp, mapFrame, odomFrame,
			tfBroadcaster_.get());
	o3d_slam::publishTfTransform(mapper_->getMapToRangeSensor(time).matrix(), timestamp,
			mapFrame, "raw_rs_o3d", tfBroadcaster_.get());
}

void WrapperRos::computeFeaturesIfReady() {
	if (submaps_->numFinishedSubmaps() > 0 && !submaps_->isComputingFeatures()) {
		computeFeaturesResult_ = std::async(std::launch::async, [this]() {
			const auto finishedSubmapIds = submaps_->popFinishedSubmapIds();
			submaps_->computeFeatures(finishedSubmapIds);
		});
	}
}
void WrapperRos::attemptLoopClosuresIfReady() {
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
void WrapperRos::loopClosureWorker() {
	while (ros::ok()) {
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

void WrapperRos::updateSubmapsAndTrajectory() {

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
	const Time latestTime = mapper_->getMapToRangeSensorBuffer().latest_time();
//	const Time lastLoopClosureTime = latestLoopClosureConstraint.timestamp_;
	const Time lastLoopClosureTime = mapper_->getMapToRangeSensorBuffer().earliest_time();

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
//		Constraint c = buildConstraint(oldConstraint.sourceSubmapIdx_, oldConstraint.targetSubmapIdx_, *submaps_,
//				true, mapperParams_.placeRecognition_.maxIcpCorrespondenceDistance_,2.0, false);
//		c.isOdometryConstraint_ = false;
//		c.isInformationMatrixValid_ = true;
		optimizationProblem_->updateLoopClosureConstraint(i, c);
		loopClosureConstraints.at(i) = c;
		std::cout << "Loop closure constraint " << i << " new transform: " << asString(c.sourceToTarget_)
				<< std::endl;
	}

	submaps_->updateAdjacencyMatrix(loopClosureConstraints);

}

void WrapperRos::mesherWorker() {
	if (mesherParams_.isComputeMesh_ && !mesherBufffer_.empty() && !mapper_->getMap().points_.empty()) {
		PointCloud map = mapper_->getMap();
		o3d_slam::MaxRadiusCroppingVolume cropper(localMapParams_.croppingRadius_);
		const auto time = mesherBufffer_.pop();
		const auto timestamp = toRos(time);
		cropper.setPose(mapper_->getMapToRangeSensor(time));
		cropper.crop(&map);
		auto downSampledMap = map.VoxelDownSample(mesherParams_.voxelSize_);
		mesher_->setCurrentPose(mapper_->getMapToRangeSensor(time));
		mesher_->buildMeshFromCloud(*downSampledMap);
		o3d_slam::publishMesh(mesher_->getMesh(), mapFrame, timestamp, meshPub_);
	}

}

void WrapperRos::publishMaps(const Time &time) {
	if (visualizationUpdateTimer_.elapsedMsec() < visualizationParameters_.visualizeEveryNmsec_
			&& !isVisualizationFirstTime_) {
		return;
	}

	const auto timestamp = toRos(time);
	std::thread t([this, timestamp]() {
		PointCloud map = mapper_->getAssembledMap();
		voxelize(visualizationParameters_.assembledMapVoxelSize_, &map);
		o3d_slam::publishCloud(map, o3d_slam::frames::mapFrame, timestamp, assembledMapPub_);
		o3d_slam::publishCloud(mapper_->getPreprocessedScan(), o3d_slam::frames::rangeSensorFrame, timestamp, mappingInputPub_);
//		o3d_slam::publishCloud(mapper_->getDenseMap(), o3d_slam::frames::mapFrame, timestamp,
//				denseMapPub_);
		o3d_slam::publishSubmapCoordinateAxes(mapper_->getSubmaps(), o3d_slam::frames::mapFrame, timestamp,
				submapOriginsPub_);
		if (submapsPub_.getNumSubscribers() > 0) {
			open3d::geometry::PointCloud cloud;
			o3d_slam::assembleColoredPointCloud(mapper_->getSubmaps(), &cloud);
			voxelize(visualizationParameters_.submapVoxelSize_, &cloud);
			o3d_slam::publishCloud(cloud, o3d_slam::frames::mapFrame, timestamp, submapsPub_);
		}
	});
	t.detach();
	visualizationUpdateTimer_.reset();
	isVisualizationFirstTime_ = false;
}

} // namespace o3d_slam

