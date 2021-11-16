/*
 * mapping_node.cpp
 *
 *  Created on: Sep 1, 2021
 *      Author: jelavice
 */
#include <open3d/Open3D.h>
#include "m545_volumetric_mapping/Parameters.hpp"
#include "m545_volumetric_mapping/frames.hpp"
#include "m545_volumetric_mapping/helpers.hpp"
#include "m545_volumetric_mapping/helpers_ros.hpp"
#include "m545_volumetric_mapping/time.hpp"
#include "m545_volumetric_mapping/math.hpp"
#include "m545_volumetric_mapping/croppers.hpp"
#include "m545_volumetric_mapping/Mapper.hpp"
#include "m545_volumetric_mapping/Mesher.hpp"
#include "m545_volumetric_mapping/OptimizationProblem.hpp"

#include "m545_volumetric_mapping/Odometry.hpp"
#include <m545_volumetric_mapping_msgs/PolygonMesh.h>

#include "open3d_conversions/open3d_conversions.h"
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

using namespace m545_mapping;
using namespace m545_mapping::frames;
open3d::geometry::PointCloud cloudPrev;
ros::NodeHandlePtr nh;
std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
ros::Publisher refPub, subsampledPub, mapPub, localMapPub, meshPub, debugPub, debugPub2, submapPub, submapOriginsPub;
std::shared_ptr<m545_mapping::Mesher> mesher;
std::shared_ptr<m545_mapping::LidarOdometry> odometry;
std::shared_ptr<m545_mapping::Mapper> mapper;
std::shared_ptr<SubmapCollection> submaps;
std::shared_ptr<OptimizationProblem> optimizationProblem;

m545_mapping::MapperParameters mapperParams;
m545_mapping::LocalMapParameters localMapParams;
m545_mapping::MesherParameters mesherParams;
double avgTime = 0.0;
int count = 0;
bool computeAndPublishOdometry(const open3d::geometry::PointCloud &cloud, const ros::Time &timestamp) {
	const Time time = fromRos(timestamp);
	odometry->addRangeScan(cloud, time);

	m545_mapping::publishTfTransform(odometry->getOdomToRangeSensor(time).matrix(), timestamp, odomFrame, rangeSensorFrame,
			tfBroadcaster.get());

	m545_mapping::publishCloud(odometry->getPreProcessedCloud(), m545_mapping::frames::rangeSensorFrame, timestamp,
			subsampledPub);

	return true;
}

void mappingUpdate(const open3d::geometry::PointCloud &cloud, const ros::Time &timestamp) {
	const Time time = fromRos(timestamp);
	{
		m545_mapping::Timer timer;
		mapper->addRangeMeasurement(cloud, time);
//		avgTime += timer.elapsedMsec();
//		++count;
//		std::cout << "Mapping step avg: " << avgTime / count << "\n";
	}
	m545_mapping::publishTfTransform(mapper->getMapToOdom(time).matrix(), timestamp, mapFrame, odomFrame,
			tfBroadcaster.get());
	m545_mapping::publishCloud(mapper->getAssembledMap(), m545_mapping::frames::mapFrame, timestamp, mapPub);

	m545_mapping::publishCloud(mapper->getActiveSubmap().toRemove_, m545_mapping::frames::mapFrame, timestamp,
			debugPub);
	m545_mapping::publishCloud(mapper->getActiveSubmap().scanRef_, m545_mapping::frames::mapFrame, timestamp,
			debugPub2);
	m545_mapping::publishCloud(mapper->getDenseMap(), m545_mapping::frames::mapFrame, timestamp, localMapPub);
	m545_mapping::publishSubmapCoordinateAxes(mapper->getSubmaps(), m545_mapping::frames::mapFrame, timestamp,
			submapOriginsPub);
	if (submapPub.getNumSubscribers() > 0) {
		open3d::geometry::PointCloud cloud;
		m545_mapping::assembleColoredPointCloud(mapper->getSubmaps(), &cloud);
		auto decimated = *(cloud.VoxelDownSample(0.2));
		m545_mapping::publishCloud(decimated, m545_mapping::frames::mapFrame, timestamp, submapPub);
	}
}

void mappingUpdateIfMapperNotBusy(const open3d::geometry::PointCloud &cloud, const ros::Time &timestamp) {
	const Time time = fromRos(timestamp);
	if (!mapper->isMatchingInProgress()) {
		std::thread t([cloud, timestamp]() {
			mappingUpdate(cloud, timestamp);
		});
		t.detach();
	}

	if (mesherParams.isComputeMesh_ && !mesher->isMeshingInProgress() && !mapper->getMap().points_.empty()) {
		std::thread t([timestamp,time]() {
			auto map = mapper->getMap();
			m545_mapping::MaxRadiusCroppingVolume cropper(localMapParams.croppingRadius_);
			cropper.setPose(mapper->getMapToRangeSensor(time));
			cropper.crop(&map);
			auto downSampledMap = map.VoxelDownSample(mesherParams.voxelSize_);
			mesher->setCurrentPose(mapper->getMapToRangeSensor(time));
			mesher->buildMeshFromCloud(*downSampledMap);
			m545_mapping::publishMesh(mesher->getMesh(), mapFrame, timestamp, meshPub);
		});
		t.detach();
	}

	if (!submaps->getFinishedSubmapIds().empty() && !submaps->isComputingFeatures()) {
		std::thread t([]() {
			submaps->computeFeatures();
		});
		t.detach();
	}

	if (!submaps->getLoopClosureCandidateIds().empty() && !submaps->isBuildingLoopClosureConstraints()) {
		std::thread t([]() {
			submaps->buildLoopClosureConstraints();
		});
		t.detach();
	}
	if (optimizationProblem->isReadyToOptimize() && !optimizationProblem->isRunningOptimization()) {
		std::thread t([]() {
			optimizationProblem->solve(*submaps);
		});
		t.detach();
	}

}

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
	open3d::geometry::PointCloud cloud;
	open3d_conversions::rosToOpen3d(msg, cloud, true);
	const ros::Time timestamp = msg->header.stamp;

	if (!computeAndPublishOdometry(cloud, timestamp)) {
		return;
	}


	if (cloudPrev.IsEmpty()) {
		cloudPrev = cloud;
		mappingUpdateIfMapperNotBusy(cloud, timestamp);
		return;
	}


	m545_mapping::publishTfTransform(Eigen::Matrix4d::Identity(), timestamp, rangeSensorFrame, msg->header.frame_id,
			tfBroadcaster.get());
	m545_mapping::publishCloud(cloud, m545_mapping::frames::rangeSensorFrame, timestamp, refPub);
	mappingUpdateIfMapperNotBusy(cloud, timestamp);

}

int main(int argc, char **argv) {
	ros::init(argc, argv, "lidar_odometry_mapping_node");
	nh.reset(new ros::NodeHandle("~"));
	tfBroadcaster.reset(new tf2_ros::TransformBroadcaster());
	const std::string cloudTopic = nh->param<std::string>("cloud_topic", "");
	ros::Subscriber cloudSub = nh->subscribe(cloudTopic, 100, &cloudCallback);

	refPub = nh->advertise<sensor_msgs::PointCloud2>("reference", 1, true);
	subsampledPub = nh->advertise<sensor_msgs::PointCloud2>("subsampled", 1, true);
	mapPub = nh->advertise<sensor_msgs::PointCloud2>("map", 1, true);
	localMapPub = nh->advertise<sensor_msgs::PointCloud2>("local_map", 1, true);
	meshPub = nh->advertise<m545_volumetric_mapping_msgs::PolygonMesh>("mesh", 1, true);
	debugPub = nh->advertise<sensor_msgs::PointCloud2>("debug", 1, true);
	debugPub2 = nh->advertise<sensor_msgs::PointCloud2>("debug2", 1, true);
	submapPub = nh->advertise<sensor_msgs::PointCloud2>("submaps", 1, true);
	submapOriginsPub = nh->advertise<visualization_msgs::MarkerArray>("submap_origins", 1, true);

	const std::string paramFile = nh->param<std::string>("parameter_file_path", "");
	std::cout << "loading params from: " << paramFile << "\n";

	m545_mapping::loadParameters(paramFile, &localMapParams);

	m545_mapping::OdometryParameters odometryParams;
	m545_mapping::loadParameters(paramFile, &odometryParams);
	odometry = std::make_shared<m545_mapping::LidarOdometry>();
	odometry->setParameters(odometryParams);

	optimizationProblem = std::make_shared<m545_mapping::OptimizationProblem>();
	submaps = std::make_shared<m545_mapping::SubmapCollection>(optimizationProblem);
	mapper = std::make_shared<m545_mapping::Mapper>(odometry->getBuffer(), submaps);
	m545_mapping::loadParameters(paramFile, &mapperParams);
	mapper->setParameters(mapperParams);

	mesher = std::make_shared<m545_mapping::Mesher>();
	m545_mapping::loadParameters(paramFile, &mesherParams);
	mesher->setParameters(mesherParams);

//	ros::AsyncSpinner spinner(4); // Use 4 threads
//	spinner.start();
//	ros::waitForShutdown();
	ros::spin();

	return 0;
}

