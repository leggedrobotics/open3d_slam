/*
 * SlamWrapperRosRos.cpp
 *
 *  Created on: Apr 19, 2022
 *      Author: jelavice
 */


#include "open3d_slam/SlamWrapperRos.hpp"

#include <chrono>
#include <open3d/Open3D.h>
#include "open3d_conversions/open3d_conversions.h"
#include <open3d_slam_msgs/PolygonMesh.h>
#include "open3d_slam/Parameters.hpp"
#include "open3d_slam/frames.hpp"
#include "open3d_slam/helpers.hpp"
#include "open3d_slam/helpers_ros.hpp"
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

#ifdef open3d_slam_OPENMP_FOUND
#include <omp.h>
#endif

namespace o3d_slam {

namespace {
using namespace o3d_slam::frames;
const double timingStatsEveryNsec = 15.0;
}

SlamWrapperRos::SlamWrapperRos(ros::NodeHandlePtr nh) : BASE(),
		nh_(nh) {
	tfBroadcaster_.reset(new tf2_ros::TransformBroadcaster());
}

void SlamWrapperRos::loadParametersAndInitialize() {

	odometryInputPub_ = nh_->advertise<sensor_msgs::PointCloud2>("odom_input", 1, true);
	mappingInputPub_ = nh_->advertise<sensor_msgs::PointCloud2>("mapping_input", 1, true);
	assembledMapPub_ = nh_->advertise<sensor_msgs::PointCloud2>("assembled_map", 1, true);
	denseMapPub_ = nh_->advertise<sensor_msgs::PointCloud2>("dense_map", 1, true);
	meshPub_ = nh_->advertise<open3d_slam_msgs::PolygonMesh>("mesh", 1, true);

	submapsPub_ = nh_->advertise<sensor_msgs::PointCloud2>("submaps", 1, true);
	submapOriginsPub_ = nh_->advertise<visualization_msgs::MarkerArray>("submap_origins", 1, true);

	saveMapSrv_ = nh_->advertiseService("save_map",&SlamWrapperRos::saveMapCallback, this);
	saveSubmapsSrv_ = nh_->advertiseService("save_submaps",&SlamWrapperRos::saveSubmapsCallback, this);

	//	auto &logger = open3d::utility::Logger::GetInstance();
	//	logger.SetVerbosityLevel(open3d::utility::VerbosityLevel::Debug);

	folderPath_ = ros::package::getPath("open3d_slam") + "/data/";
	mapSavingFolderPath_ = nh_->param<std::string>("map_saving_folder", folderPath_);
	const std::string paramFile = nh_->param<std::string>("parameter_file_path", "");
	setParameterFilePath(paramFile);
	BASE::loadParametersAndInitialize();
}

bool SlamWrapperRos::saveMapCallback(open3d_slam_msgs::SaveMap::Request &req,
		open3d_slam_msgs::SaveMap::Response &res) {
	const bool savingResult = saveMap(mapSavingFolderPath_);
	res.statusMessage = savingResult ? "Map saved to: " + mapSavingFolderPath_ : "Error while saving map";
	return true;
}
bool SlamWrapperRos::saveSubmapsCallback(open3d_slam_msgs::SaveSubmaps::Request &req,open3d_slam_msgs::SaveSubmaps::Response &res){
	const bool savingResult =saveSubmaps(mapSavingFolderPath_);
	res.statusMessage = savingResult ? "Submaps saved to: " + mapSavingFolderPath_ : "Error while saving submaps";
	return true;
}

void SlamWrapperRos::publishMapToOdomTf(const Time &time) {
	const auto timestamp = toRos(time);
	o3d_slam::publishTfTransform(mapper_->getMapToOdom(time).matrix(), timestamp, mapFrame, odomFrame,
			tfBroadcaster_.get());
	o3d_slam::publishTfTransform(mapper_->getMapToRangeSensor(time).matrix(), timestamp,
			mapFrame, "raw_rs_o3d", tfBroadcaster_.get());
}

void SlamWrapperRos::publishDenseMap(const Time &time) {
	if (denseMapVisualizationUpdateTimer_.elapsedMsec() < visualizationParameters_.visualizeEveryNmsec_) {
		return;
	}
	if (isPublishDenseMapThreadRunning_) {
		return;
	}

	isPublishDenseMapThreadRunning_ = true;
	std::thread t([this, time]() {
		const auto denseMap = mapper_->getDenseMap(); //copy
		const auto timestamp = toRos(time);
		o3d_slam::publishCloud(denseMap.toPointCloud(), o3d_slam::frames::mapFrame, timestamp, denseMapPub_);
		denseMapVisualizationUpdateTimer_.reset();
		isPublishDenseMapThreadRunning_ = false;
	});
	t.detach();

}

void SlamWrapperRos::publishMaps(const Time &time) {
	if (visualizationUpdateTimer_.elapsedMsec() < visualizationParameters_.visualizeEveryNmsec_
			&& !isVisualizationFirstTime_) {
		return;
	}

	if (isPublishMapsThreadRunning_){
		return;
	}

	const auto timestamp = toRos(time);
	isPublishMapsThreadRunning_ = true;
	std::thread t([this, timestamp]() {
		PointCloud map = mapper_->getAssembledMapPointCloud();
		voxelize(visualizationParameters_.assembledMapVoxelSize_, &map);
		o3d_slam::publishCloud(map, o3d_slam::frames::mapFrame, timestamp, assembledMapPub_);
		o3d_slam::publishCloud(mapper_->getPreprocessedScan(), o3d_slam::frames::rangeSensorFrame, timestamp, mappingInputPub_);
		o3d_slam::publishSubmapCoordinateAxes(mapper_->getSubmaps(), o3d_slam::frames::mapFrame, timestamp,
				submapOriginsPub_);
		if (submapsPub_.getNumSubscribers() > 0) {
			open3d::geometry::PointCloud cloud;
			o3d_slam::assembleColoredPointCloud(mapper_->getSubmaps(), &cloud);
			voxelize(visualizationParameters_.submapVoxelSize_, &cloud);
			o3d_slam::publishCloud(cloud, o3d_slam::frames::mapFrame, timestamp, submapsPub_);
		}
		isPublishMapsThreadRunning_ = false;
	});
	t.detach();
	visualizationUpdateTimer_.reset();
	isVisualizationFirstTime_ = false;
}

} // namespace o3d_slam


