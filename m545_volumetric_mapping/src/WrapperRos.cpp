/*
 * WrapperRos.cpp
 *
 *  Created on: Nov 23, 2021
 *      Author: jelavice
 */

#include "m545_volumetric_mapping/WrapperRos.hpp"

#include <chrono>
#include <open3d/Open3D.h>
#include "m545_volumetric_mapping/Parameters.hpp"
#include "m545_volumetric_mapping/frames.hpp"
#include "m545_volumetric_mapping/helpers.hpp"
#include "m545_volumetric_mapping/helpers_ros.hpp"
#include "m545_volumetric_mapping/time.hpp"
#include "m545_volumetric_mapping/math.hpp"
#include "m545_volumetric_mapping/croppers.hpp"
#include "m545_volumetric_mapping/Mapper.hpp"
#include "m545_volumetric_mapping/assert.hpp"
#include "m545_volumetric_mapping/Mesher.hpp"
#include "m545_volumetric_mapping/OptimizationProblem.hpp"

#include "m545_volumetric_mapping/Odometry.hpp"
#include "open3d_conversions/open3d_conversions.h"
#include <m545_volumetric_mapping_msgs/PolygonMesh.h>

namespace m545_mapping {

namespace {
using namespace m545_mapping::frames;
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

void WrapperRos::initialize() {

	odometryInputPub_ = nh_->advertise<sensor_msgs::PointCloud2>("odom_input", 1, true);
	mappingInputPub_ = nh_->advertise<sensor_msgs::PointCloud2>("mapping_input", 1, true);
	assembledMapPub_ = nh_->advertise<sensor_msgs::PointCloud2>("assembled_map", 1, true);
	denseMapPub_ = nh_->advertise<sensor_msgs::PointCloud2>("dense_map", 1, true);
	meshPub_ = nh_->advertise<m545_volumetric_mapping_msgs::PolygonMesh>("mesh", 1, true);

	submapsPub_ = nh_->advertise<sensor_msgs::PointCloud2>("submaps", 1, true);
	submapOriginsPub_ = nh_->advertise<visualization_msgs::MarkerArray>("submap_origins", 1, true);

	//	auto &logger = open3d::utility::Logger::GetInstance();
	//	logger.SetVerbosityLevel(open3d::utility::VerbosityLevel::Debug);

	folderPath_ = ros::package::getPath("m545_volumetric_mapping") + "/data/";

	const std::string paramFile = nh_->param<std::string>("parameter_file_path", "");
	std::cout << "loading params from: " << paramFile << "\n";

	m545_mapping::loadParameters(paramFile, &localMapParams_);

	OdometryParameters odometryParams;
	loadParameters(paramFile, &odometryParams);
	odometry_ = std::make_shared<m545_mapping::LidarOdometry>();
	odometry_->setParameters(odometryParams);

	submaps_ = std::make_shared<m545_mapping::SubmapCollection>();
	mapper_ = std::make_shared<m545_mapping::Mapper>(odometry_->getBuffer(), submaps_);
	m545_mapping::loadParameters(paramFile, &mapperParams_);
	mapper_->setParameters(mapperParams_);

	optimizationProblem_ = std::make_shared<m545_mapping::OptimizationProblem>();
	optimizationProblem_->setParameters(mapperParams_);

	mesher_ = std::make_shared<m545_mapping::Mesher>();
	m545_mapping::loadParameters(paramFile, &mesherParams_);
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
		m545_mapping::publishTfTransform(odometry_->getOdomToRangeSensor(measurement.time_).matrix(), timestamp,
				odomFrame, rangeSensorFrame, tfBroadcaster_.get());
		m545_mapping::publishTfTransform(odometry_->getOdomToRangeSensor(measurement.time_).matrix(), timestamp,
				mapFrame, "raw_odom", tfBroadcaster_.get());

		if (odometryInputPub_.getNumSubscribers() > 0) {
			auto odomInput = odometry_->getPreProcessedCloud();
			std::thread t(
					[this, timestamp, odomInput]() {
						m545_mapping::publishCloud(odomInput, m545_mapping::frames::rangeSensorFrame, timestamp,
								odometryInputPub_);
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
			submaps_->dumpToFile(folderPath_, "after");
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

		const auto denseMap = mapper_->getDenseMap();
		if (denseMap.HasColors()) {
			const auto timestamp = toRos(regCloud.raw_.time_);
			m545_mapping::publishCloud(denseMap, m545_mapping::frames::mapFrame, timestamp, denseMapPub_);
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
	m545_mapping::publishTfTransform(mapper_->getMapToOdom(time).matrix(), timestamp, mapFrame, odomFrame,
			tfBroadcaster_.get());
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
	static bool isFirstTime = true;
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

		if (!isFirstTime || loopClosureConstraints.empty()) {
			std::this_thread::sleep_for(std::chrono::milliseconds(200));
			continue;
		}
		isFirstTime = false;
		{
			Timer t("optimization_problem");
			auto odometryConstraints = submaps_->getOdometryConstraints();
			computeOdometryConstraints(*submaps_, &odometryConstraints);
			submaps_->addLoopClosureConstraints(loopClosureConstraints);

			optimizationProblem_->clearLoopClosureConstraints();
			optimizationProblem_->clearOdometryConstraints();
			optimizationProblem_->insertLoopClosureConstraints(loopClosureConstraints);
			optimizationProblem_->insertOdometryConstraints(odometryConstraints);
			optimizationProblem_->buildOptimizationProblem(*submaps_);

//			optimizationProblem_->print();
			submaps_->dumpToFile(folderPath_, "before");
			optimizationProblem_->dumpToFile(folderPath_ + "/poseGraph.json");
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
	assert_gt(latestLoopClosureConstraint.sourceSubmapIdx_, latestLoopClosureConstraint.targetSubmapIdx_);
	const auto dT = optimizedTransformations.at(latestLoopClosureConstraint.sourceSubmapIdx_);
	const Time latestTime = mapper_->getMapToRangeSensorBuffer().latest_time();
//	const Time lastLoopClosureTime = latestLoopClosureConstraint.timestamp_;
	const Time lastLoopClosureTime = mapper_->getMapToRangeSensorBuffer().earliest_time();

	std::cout << "Transforming the pose buffer with the delta T from submap "
			<< latestLoopClosureConstraint.sourceSubmapIdx_ << "the transform is: \n" << asString(dT.dT_)
			<< std::endl;
	auto mapToRangeSensorBufferPtr = mapper_->getMapToRangeSensorBufferPtr();
	mapToRangeSensorBufferPtr->applyToAllElementsInTimeInterval(dT.dT_, lastLoopClosureTime, latestTime);
	const auto updatedMapToRangeSensor = mapper_->getMapToRangeSensorBuffer().lookup(latestTime);
	mapper_->setMapToRangeSensor(updatedMapToRangeSensor);
	submaps_->setMapToRangeSensor(updatedMapToRangeSensor);

}

void WrapperRos::mesherWorker() {
	if (mesherParams_.isComputeMesh_ && !mesherBufffer_.empty() && !mapper_->getMap().points_.empty()) {
		auto map = mapper_->getMap();
		m545_mapping::MaxRadiusCroppingVolume cropper(localMapParams_.croppingRadius_);
		const auto time = mesherBufffer_.pop();
		const auto timestamp = toRos(time);
		cropper.setPose(mapper_->getMapToRangeSensor(time));
		cropper.crop(&map);
		auto downSampledMap = map.VoxelDownSample(mesherParams_.voxelSize_);
		mesher_->setCurrentPose(mapper_->getMapToRangeSensor(time));
		mesher_->buildMeshFromCloud(*downSampledMap);
		m545_mapping::publishMesh(mesher_->getMesh(), mapFrame, timestamp, meshPub_);
	}

}

void WrapperRos::publishMaps(const Time &time) {
	if (visualizationUpdateTimer_.elapsedMsec() < visualizationParameters_.visualizeEveryNmsec_
			&& !isVisualizationFirstTime_) {
		return;
	}

	const auto timestamp = toRos(time);
	std::thread t([this, timestamp]() {
		auto map = mapper_->getAssembledMap();
		voxelize(visualizationParameters_.assembledMapVoxelSize_, &map);
		m545_mapping::publishCloud(map, m545_mapping::frames::mapFrame, timestamp, assembledMapPub_);
		m545_mapping::publishCloud(mapper_->getPreprocessedScan(), m545_mapping::frames::rangeSensorFrame, timestamp,
	mappingInputPub_);
//		m545_mapping::publishCloud(mapper_->getDenseMap(), m545_mapping::frames::mapFrame, timestamp,
//				denseMapPub_);
		m545_mapping::publishSubmapCoordinateAxes(mapper_->getSubmaps(), m545_mapping::frames::mapFrame,
				timestamp, submapOriginsPub_);
		if (submapsPub_.getNumSubscribers() > 0) {
			open3d::geometry::PointCloud cloud;
			m545_mapping::assembleColoredPointCloud(mapper_->getSubmaps(), &cloud);
			voxelize(visualizationParameters_.submapVoxelSize_, &cloud);
			m545_mapping::publishCloud(cloud, m545_mapping::frames::mapFrame, timestamp, submapsPub_);
		}
	});
	t.detach();
	visualizationUpdateTimer_.reset();
	isVisualizationFirstTime_ = false;
}

} // namespace m545_mapping

