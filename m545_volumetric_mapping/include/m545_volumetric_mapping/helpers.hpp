/*
 * helpers.hpp
 *
 *  Created on: Sep 26, 2021
 *      Author: jelavice
 */

#pragma once
#include <chrono>
#include <open3d/geometry/PointCloud.h>
#include <open3d/geometry/MeshBase.h>

#include <open3d/pipelines/registration/TransformationEstimation.h>
#include "m545_volumetric_mapping/Parameters.hpp"

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/publisher.h>
#include <tf2_ros/transform_broadcaster.h>


namespace m545_mapping {

std::shared_ptr<open3d::geometry::PointCloud> voxelizeAroundPosition(double voxelSize, const open3d::geometry::AxisAlignedBoundingBox &bbox, const open3d::geometry::PointCloud &cloud);
void cropPointcloud(const open3d::geometry::AxisAlignedBoundingBox &bbox, open3d::geometry::PointCloud *pcl);
void randomDownSample(double downSamplingRatio, open3d::geometry::PointCloud *pcl);
void voxelize(double voxelSize, open3d::geometry::PointCloud *pcl);

void estimateNormals(int numNearestNeighbours, open3d::geometry::PointCloud *pcl);
std::shared_ptr<open3d::pipelines::registration::TransformationEstimation> icpObjectiveFactory(
		const m545_mapping::IcpObjective &obj);

std::string asString (const Eigen::Isometry3d &T);

void publishCloud(const open3d::geometry::PointCloud &cloud, const std::string &frame_id, const ros::Time &timestamp,ros::Publisher &pub);
void publishMesh(const open3d::geometry::MeshBase &mesh, const std::string &frame_id, const ros::Time &timestamp,ros::Publisher &pub);

bool isInside(const open3d::geometry::AxisAlignedBoundingBox &bbox, const Eigen::Vector3d &p);
geometry_msgs::Pose getPose(const Eigen::MatrixXd &T);

geometry_msgs::TransformStamped toRos(const Eigen::Matrix4d &Mat, const ros::Time &time, const std::string &frame,
		const std::string &childFrame);

open3d::geometry::AxisAlignedBoundingBox boundingBoxAroundPosition(const Eigen::Vector3d &low,const Eigen::Vector3d &high, const Eigen::Vector3d &origin = Eigen::Vector3d::Zero());

std::pair<std::vector<double>, std::vector<size_t>> computePointCloudDistance(const open3d::geometry::PointCloud &reference,
		const open3d::geometry::PointCloud &cloud, const std::vector<size_t> &idsInReference);



void publishTfTransform(const Eigen::Matrix4d &Mat, const ros::Time &time, const std::string &frame,
		const std::string &childFrame, tf2_ros::TransformBroadcaster *broadcaster);


void removeByIds(const std::vector<size_t> &ids, open3d::geometry::PointCloud *cloud);


} /* namespace m545_mapping */
