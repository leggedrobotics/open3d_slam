/*
 * mapping_node.cpp
 *
 *  Created on: Sep 1, 2021
 *      Author: jelavice
 */
#include <open3d/Open3D.h>
#include <open3d/pipelines/registration/Registration.h>
#include <open3d/geometry/Image.h>
#include <open3d/geometry/PointCloud.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "m545_volumetric_mapping/Parameters.hpp"
#include "m545_volumetric_mapping/frames.hpp"
#include "m545_volumetric_mapping/helpers.hpp"
#include "m545_volumetric_mapping/Mapper.hpp"
#include "m545_volumetric_mapping/Mesher.hpp"
#include "m545_volumetric_mapping/Projection.hpp"
#include "m545_volumetric_mapping/CvImage.hpp"
#include "m545_volumetric_mapping/Odometry.hpp"
#include <m545_volumetric_mapping_msgs/PolygonMesh.h>

#include "open3d_conversions/open3d_conversions.h"
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>

using namespace m545_mapping::frames;
open3d::geometry::PointCloud cloudPrev;
ros::NodeHandlePtr nh;
std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
ros::Publisher refPub, subsampledPub, mapPub, localMapPub, meshPub, colorCloudPub;
std::shared_ptr<m545_mapping::Mesher> mesher;
std::shared_ptr<m545_mapping::LidarOdometry> odometry;
std::shared_ptr<m545_mapping::Mapper> mapper;
m545_mapping::MapperParameters mapperParams;
m545_mapping::LocalMapParameters localMapParams;
m545_mapping::MesherParameters mesherParams;
m545_mapping::ProjectionParameters projectionParams;


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
	}
	m545_mapping::publishTfTransform(mapper->getMapToOdom().matrix(), timestamp, mapFrame, odomFrame,
			tfBroadcaster.get());

	open3d::geometry::PointCloud map = mapper->getMap();
	m545_mapping::publishCloud(mapper->getMap(), m545_mapping::frames::mapFrame, timestamp, mapPub);

	if (localMapPub.getNumSubscribers() > 0) {
		open3d::geometry::PointCloud map = mapper->getDenseMap();
		auto bbox = m545_mapping::boundingBoxAroundPosition(localMapParams.cropBoxLowBound_,
				localMapParams.cropBoxHighBound_, mapper->getMapToRangeSensor().translation());
		m545_mapping::cropPointcloud(bbox, &map);
		auto downSampledMap = map.VoxelDownSample(localMapParams.voxelSize_);
		m545_mapping::publishCloud(*downSampledMap, m545_mapping::frames::mapFrame, timestamp, localMapPub);
	}
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
			auto bbox = m545_mapping::boundingBoxAroundPosition(localMapParams.cropBoxLowBound_, localMapParams.cropBoxHighBound_, mapper->getMapToRangeSensor().translation());
			m545_mapping::cropPointcloud(bbox, &map);
			auto downSampledMap = map.VoxelDownSample(mesherParams.voxelSize_);
			mesher->setCurrentPose(mapper->getMapToRangeSensor());
			mesher->buildMeshFromCloud(*downSampledMap);
			m545_mapping::publishMesh(mesher->getMesh(), mapFrame,timestamp,meshPub);
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

//yidan
void synchronizeCallback(const sensor_msgs::PointCloud2ConstPtr& cloudmsg, const sensor_msgs::ImageConstPtr& imagemsg) {

    open3d::geometry::PointCloud pointCloud;
    sensor_msgs::PointCloud2 colorCloud;
    open3d_conversions::rosToOpen3d(cloudmsg, pointCloud, true);
    ros::Time timestamp = cloudmsg->header.stamp;
    std::vector<Eigen::Vector2i> pixels(pointCloud.points_.size());
    Eigen::Matrix<double, 3, 1> zeroPoint = Eigen::Vector3d::Zero();

//    std::vector<Eigen::Matrix<double, 3, 1>> pointCloud_points_crop;
//
//    for (int i = 0; i < pointCloud.points_.size(); i++) {
//        if(pointCloud.points_[i] != zeroPoint) {
//            pointCloud_points_crop.push_back(pointCloud.points_[i]);
//        }
//    }
    pixels = projectionLidarToPixel(pointCloud.points_,  projectionParams.K, projectionParams.D, projectionParams.quaternion, projectionParams.translation);
    pointCloud.colors_ = imageConversion(imagemsg, pixels, sensor_msgs::image_encodings::RGB8);
//    std::cout << "cloudsize" << pointCloud.colors_.size() <<std::endl;
//    for (int i = 0; i < pointCloud.points_.size(); i++) {
//        std::cout << "color:" << pointCloud.colors_[i] << std::endl;
//    }
//    std::cout << "colors" << pointCloud.colors_[10] <<std::endl;

    m545_mapping::publishCloud(pointCloud, m545_mapping::frames::rangeSensorFrame, timestamp, colorCloudPub);
    mappingUpdateIfMapperNotBusy(pointCloud, timestamp);

}


int main(int argc, char **argv) {
	ros::init(argc, argv, "lidar_odometry_mapping_node");
	nh.reset(new ros::NodeHandle("~"));
	tfBroadcaster.reset(new tf2_ros::TransformBroadcaster());
	const std::string cloudTopic = nh->param<std::string>("cloud_topic", "");
	ros::Subscriber cloudSub = nh->subscribe(cloudTopic, 100, &cloudCallback);

    //yidan
    const std::string paramFile = nh->param<std::string>("parameter_file_path", "");
    std::cout << "loading params from: " << paramFile << "\n";
    m545_mapping::loadParameters(paramFile, &projectionParams);
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(*nh, "/ouster_points_self_filtered", 1);
    message_filters::Subscriber<sensor_msgs::Image> image_sub(*nh, "/camMainView/Downsampled", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), cloud_sub, image_sub);
    sync.registerCallback(boost::bind(&synchronizeCallback, _1, _2));

	refPub = nh->advertise<sensor_msgs::PointCloud2>("reference", 1, true);
	subsampledPub = nh->advertise<sensor_msgs::PointCloud2>("subsampled", 1, true);
	mapPub = nh->advertise<sensor_msgs::PointCloud2>("map", 1, true);
	localMapPub = nh->advertise<sensor_msgs::PointCloud2>("local_map", 1, true);
	meshPub = nh->advertise<m545_volumetric_mapping_msgs::PolygonMesh>("mesh", 1, true);
    colorCloudPub = nh->advertise<sensor_msgs::PointCloud2>("color_cloud", 1, true);

	m545_mapping::OdometryParameters odometryParams;
	m545_mapping::loadParameters(paramFile, &odometryParams);
	odometry = std::make_shared<m545_mapping::LidarOdometry>();
	odometry->setParameters(odometryParams);

	mapper = std::make_shared<m545_mapping::Mapper>();
	m545_mapping::loadParameters(paramFile, &mapperParams);
	mapper->setParameters(mapperParams);

	m545_mapping::loadParameters(paramFile, &localMapParams);

	mesher = std::make_shared<m545_mapping::Mesher>();
	m545_mapping::loadParameters(paramFile, &mesherParams);
	mesher->setParameters(mesherParams);

//	ros::AsyncSpinner spinner(4); // Use 4 threads
//	spinner.start();
//	ros::waitForShutdown();
	ros::spin();

	return 0;
}

