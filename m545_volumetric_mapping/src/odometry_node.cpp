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
#include <message_filters/time_synchronizer.h>

#include "m545_volumetric_mapping/Parameters.hpp"
#include "m545_volumetric_mapping/frames.hpp"
#include "m545_volumetric_mapping/helpers.hpp"
#include "m545_volumetric_mapping/Mapper.hpp"
#include "m545_volumetric_mapping/Projection.hpp"

#include "open3d_conversions/open3d_conversions.h"
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

open3d::geometry::PointCloud cloudPrev;

ros::NodeHandlePtr nh;
ros::NodeHandle nh2;

std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
Eigen::Matrix4d curentTransformation = Eigen::Matrix4d::Identity();
namespace registration = open3d::pipelines::registration;
ros::Publisher refPub;
ros::Publisher subsampledPub;
ros::Publisher mapPub, localMapPub;

m545_mapping::OdometryParameters odometryParams;
std::shared_ptr<registration::TransformationEstimation> icpObjective;
std::shared_ptr<m545_mapping::Mapper> mapper;
m545_mapping::MapperParameters mapperParams;
m545_mapping::LocalMapParameters localMapParams;

bool computeAndPublishOdometry(const open3d::geometry::PointCloud &cloud, const ros::Time &timestamp) {
	const m545_mapping::Timer timer;
	const Eigen::Matrix4d init = Eigen::Matrix4d::Identity();
	auto criteria = open3d::pipelines::registration::ICPConvergenceCriteria();
	criteria.max_iteration_ = odometryParams.maxNumIter_;
	auto bbox = m545_mapping::boundingBoxAroundPosition(odometryParams.cropBoxLowBound_,
			odometryParams.cropBoxHighBound_);
	auto croppedCloud = cloud.Crop(bbox);
//	auto voxelizedCloud = croppedCloud->VoxelDownSample(odometryParams.voxelSize_);
	m545_mapping::voxelize(odometryParams.voxelSize_, croppedCloud.get());
	auto downSampledCloud = croppedCloud->RandomDownSample(odometryParams.downSamplingRatio_);

	if (odometryParams.icpObjective_ == m545_mapping::IcpObjective::PointToPlane) {
		m545_mapping::estimateNormals(odometryParams.kNNnormalEstimation_, downSampledCloud.get());
		downSampledCloud->NormalizeNormals();
	}
	auto result = open3d::pipelines::registration::RegistrationICP(cloudPrev, *downSampledCloud,
			odometryParams.maxCorrespondenceDistance_, init, *icpObjective, criteria);

//	std::cout << "Scan to scan matching finished \n";
//	std::cout << "Time elapsed: " << timer.elapsedMsec() << " msec \n";
//	std::cout << "Fitness: " << result.fitness_ << "\n";
//	std::cout << "RMSE: " << result.inlier_rmse_ << "\n";
//	std::cout << "Transform: " << result.transformation_ << "\n";
//	std::cout << "target size: " << cloud.points_.size() << std::endl;
//	std::cout << "reference size: " << cloudPrev.points_.size() << std::endl;
//	std::cout << "\n \n";
	if (result.fitness_ <= 1e-2) {
		return false;
	}
	curentTransformation *= result.transformation_.inverse();
	geometry_msgs::TransformStamped transformStamped = m545_mapping::toRos(curentTransformation, timestamp,
			m545_mapping::frames::odomFrame, m545_mapping::frames::rangeSensorFrame);
	tfBroadcaster->sendTransform(transformStamped);
	cloudPrev = *downSampledCloud;
	m545_mapping::publishCloud(*downSampledCloud, m545_mapping::frames::rangeSensorFrame, timestamp, subsampledPub);

	return true;
}

void mappingUpdate(const open3d::geometry::PointCloud &cloud, const ros::Time &timestamp) {

	const m545_mapping::Timer timer;
	mapper->addRangeMeasurement(cloud, timestamp);
	std::cout << "Mapping step finished \n";
	std::cout << "Time elapsed: " << timer.elapsedMsec() << " msec \n\n";
	{
		geometry_msgs::TransformStamped transformStamped = m545_mapping::toRos(mapper->getMapToOdom().matrix(),
				timestamp, m545_mapping::frames::mapFrame, m545_mapping::frames::odomFrame);
		tfBroadcaster->sendTransform(transformStamped);
	}
//		{
//			geometry_msgs::TransformStamped transformStamped = toRos(mapper->getMapToRangeSensor().matrix(), timestamp,
//					m545_mapping::frames::mapFrame, m545_mapping::frames::rangeSensorFrame+"_check");
//			tfBroadcaster->sendTransform(transformStamped);
//		}

	if (mapPub.getNumSubscribers() > 0) {
		open3d::geometry::PointCloud map = mapper->getMap();
		m545_mapping::publishCloud(map, m545_mapping::frames::mapFrame, timestamp, mapPub);
	}

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
	if (mapper->isMatchingInProgress()) {
		return;
	}
	std::thread t([cloud, timestamp]() {
		mappingUpdate(cloud, timestamp);
	});
	t.detach();
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
	m545_mapping::publishCloud(cloud, m545_mapping::frames::rangeSensorFrame, timestamp, refPub);
	mappingUpdateIfMapperNotBusy(cloud, timestamp);

}


//yidan
void synchronizeCallback(const sensor_msgs::PointCloud2Ptr& cloudmsg, const sensor_msgs::Image& imagemsg) {
	open3d::geometry::PointCloud pointCloud;
	open3d_conversions::rosToOpen3d(cloudmsg, pointCloud, true);

	for (int i = 0; i < pointCloud.points_.size(); ++i) {
		Eigen::Vector3d point;
		Eigen::Vector2i pixel = Eigen::Vector2i::Zero();
		uint8_t rgb;
		//uint8_t* rgbData;
		//Eigen::Vector3f color = {1.0, 1.0, 1.0};
		
		point(0) = pointCloud.points_[i].x();
		point(1) = pointCloud.points_[i].y();
		point(2) = pointCloud.points_[i].z();
		pixel = projectionLidarToPixel(point, quaternion, translation);
		if (pixel(0) > 0 && pixel(0) < 3440 && pixel(1) > 0 && pixel(1) < 2880) {
			rgb = imagemsg.data[pixel(0), pixel(1)];
			pointCloud.colors_[i] = Eigen::Vector3d(rgb);
			// for (int i = 0; i < 3; ++i) {
			// 	rgb[1];
			// 	//color(i) = float(rgb[i] / 255.0);
			// }
		}
		//pointCloud.colors_[i] = Eigen::Vector3d(rgb);
	}
}


int main(int argc, char **argv) {
	ros::init(argc, argv, "lidar_odometry_mapping_node");
	nh.reset(new ros::NodeHandle("~"));
	tfBroadcaster.reset(new tf2_ros::TransformBroadcaster());
	const std::string cloudTopic = nh->param<std::string>("cloud_topic", "");
	ros::Subscriber cloudSub = nh->subscribe(cloudTopic, 100, &cloudCallback);

	//yidan
	message_filters::Subscriber<sensor_msgs::PointCloud2Ptr> cloud_sub(nh2, "/ouster_points_self_filtered", 1);
	message_filters::Subscriber<sensor_msgs::Image> image_sub(nh2, "/camMainView/Downsampled", 1);
	message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::PointCloud2Ptr> synchro(image_sub, cloud_sub, 10);
	synchro.registerCallback(boost::bind(&synchronizeCallback, _1, _2));

	refPub = nh->advertise<sensor_msgs::PointCloud2>("reference", 1, true);
	subsampledPub = nh->advertise<sensor_msgs::PointCloud2>("subsampled", 1, true);
	mapPub = nh->advertise<sensor_msgs::PointCloud2>("map", 1, true);
	localMapPub = nh->advertise<sensor_msgs::PointCloud2>("local_map", 1, true);

	const std::string paramFile = nh->param<std::string>("parameter_file_path", "");
	std::cout << "loading params from: " << paramFile << "\n";
	m545_mapping::loadParameters(paramFile, &odometryParams);
	icpObjective = m545_mapping::icpObjectiveFactory(odometryParams.icpObjective_);

	mapper = std::make_shared<m545_mapping::Mapper>();
	m545_mapping::loadParameters(paramFile, &mapperParams);
	mapper->setParameters(mapperParams);

	m545_mapping::loadParameters(paramFile, &localMapParams);

//	ros::AsyncSpinner spinner(4); // Use 4 threads
//	spinner.start();
//	ros::waitForShutdown();
	ros::spin();

	return 0;
}

