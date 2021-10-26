/*
 * mapping_node.cpp
 *
 *  Created on: Sep 1, 2021
 *      Author: jelavice
 */
#include <open3d/Open3D.h>
#include <open3d/pipelines/registration/Registration.h>
#include "m545_volumetric_mapping/Parameters.hpp"
#include "m545_volumetric_mapping/frames.hpp"
#include "m545_volumetric_mapping/helpers.hpp"
#include "m545_volumetric_mapping/helpers_ros.hpp"
#include "m545_volumetric_mapping/time.hpp"
#include "m545_volumetric_mapping/math.hpp"
#include "m545_volumetric_mapping/croppers.hpp"
#include "m545_volumetric_mapping/Mapper.hpp"
#include "m545_volumetric_mapping/Mesher.hpp"

#include "m545_volumetric_mapping/Odometry.hpp"
#include <m545_volumetric_mapping_msgs/PolygonMesh.h>

#include "open3d_conversions/open3d_conversions.h"
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

using namespace m545_mapping::frames;
open3d::geometry::PointCloud cloudPrev;
ros::NodeHandlePtr nh;
std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
ros::Publisher refPub, subsampledPub, mapPub, localMapPub, meshPub, debugPub, debugPub2, debugPub3;
std::shared_ptr<m545_mapping::Mesher> mesher;
std::shared_ptr<m545_mapping::LidarOdometry> odometry;
std::shared_ptr<m545_mapping::Mapper> mapper;
m545_mapping::MapperParameters mapperParams;
m545_mapping::LocalMapParameters localMapParams;
m545_mapping::MesherParameters mesherParams;

bool computeAndPublishOdometry(const open3d::geometry::PointCloud &cloud, const ros::Time &timestamp) {
	odometry->addRangeScan(cloud, timestamp);

	m545_mapping::publishTfTransform(odometry->getOdomToRangeSensor().matrix(), timestamp, odomFrame, rangeSensorFrame,
			tfBroadcaster.get());

	m545_mapping::publishCloud(odometry->getPreProcessedCloud(), m545_mapping::frames::rangeSensorFrame, timestamp,
			subsampledPub);

	return true;
}

void mappingUpdate(const open3d::geometry::PointCloud &cloud, const ros::Time &timestamp) {
	{
//		m545_mapping::Timer timer("Mapping step.");
		mapper->addRangeMeasurement(cloud, timestamp);
//		std::cout << "\n";
	}
	m545_mapping::publishTfTransform(mapper->getMapToOdom().matrix(), timestamp, mapFrame, odomFrame,
			tfBroadcaster.get());

	m545_mapping::publishCloud(mapper->getMap(), m545_mapping::frames::mapFrame, timestamp, mapPub);

	m545_mapping::publishCloud(mapper->toRemove_, m545_mapping::frames::mapFrame, timestamp, debugPub);
	m545_mapping::publishCloud(mapper->scanRef_, m545_mapping::frames::mapFrame, timestamp, debugPub2);
	m545_mapping::publishCloud(mapper->mapRef_, m545_mapping::frames::mapFrame, timestamp, debugPub3);

	m545_mapping::publishCloud(mapper->getDenseMap(), m545_mapping::frames::mapFrame, timestamp, localMapPub);
}

void mappingUpdateIfMapperNotBusy(const open3d::geometry::PointCloud &cloud, const ros::Time &timestamp) {
	if (!mapper->isMatchingInProgress()) {
		std::thread t([cloud, timestamp]() {
			mappingUpdate(cloud, timestamp);
		});
		t.detach();
	}

	if (mesherParams.isComputeMesh_ && !mesher->isMeshingInProgress() && !mapper->getMap().points_.empty()) {
		std::thread t([timestamp]() {
			auto map = mapper->getMap();
			m545_mapping::MaxRadiusCroppingVolume cropper(localMapParams.croppingRadius_);
			cropper.setPose(mapper->getMapToRangeSensor());
			cropper.crop(&map);
			auto downSampledMap = map.VoxelDownSample(mesherParams.voxelSize_);
			mesher->setCurrentPose(mapper->getMapToRangeSensor());
			mesher->buildMeshFromCloud(*downSampledMap);
			m545_mapping::publishMesh(mesher->getMesh(), mapFrame, timestamp, meshPub);
		});
		t.detach();
	}

}

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
	open3d::geometry::PointCloud cloud;
	open3d_conversions::rosToOpen3d(msg, cloud, true);
	ros::Time timestamp = msg->header.stamp;

	if (cloudPrev.IsEmpty()) {
		cloudPrev = cloud;
		mappingUpdateIfMapperNotBusy(cloud, timestamp);
		return;
	}

	if (!computeAndPublishOdometry(cloud, timestamp)) {
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
	debugPub3 = nh->advertise<sensor_msgs::PointCloud2>("debug3", 1, true);

	const std::string paramFile = nh->param<std::string>("parameter_file_path", "");
	std::cout << "loading params from: " << paramFile << "\n";

	m545_mapping::OdometryParameters odometryParams;
	m545_mapping::loadParameters(paramFile, &odometryParams);
	odometry = std::make_shared<m545_mapping::LidarOdometry>();
	odometry->setParameters(odometryParams);

	mapper = std::make_shared<m545_mapping::Mapper>();
	m545_mapping::loadParameters(paramFile, &mapperParams);
	m545_mapping::SpaceCarvingParameters carvingParams;
	m545_mapping::loadParameters(paramFile, &carvingParams);
	m545_mapping::loadParameters(paramFile, &localMapParams);

	mapper->setParameters(mapperParams, carvingParams, localMapParams);

	mesher = std::make_shared<m545_mapping::Mesher>();
	m545_mapping::loadParameters(paramFile, &mesherParams);
	mesher->setParameters(mesherParams);

//	ros::AsyncSpinner spinner(4); // Use 4 threads
//	spinner.start();
//	ros::waitForShutdown();
	ros::spin();

	return 0;
}

