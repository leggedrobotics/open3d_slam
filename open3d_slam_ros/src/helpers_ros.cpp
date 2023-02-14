/*
 * helpers_ros.cpp
 *
 *  Created on: Oct 20, 2021
 *      Author: jelavice
 */

#include "open3d_slam_ros/helpers_ros.hpp"
#include "open3d_slam/SubmapCollection.hpp"
#include <random>
// ros stuff
#include "open3d_conversions/open3d_conversions.h"
#include <eigen_conversions/eigen_msg.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <nav_msgs/Odometry.h>
#include "open3d_slam_ros/Color.hpp"
#include "open3d_slam/frames.hpp"

namespace o3d_slam {
int id = 0;

void jetColormap(int value, int min_value, int max_value, float& r, float& g, float& b) {
	float v = static_cast<float>(value - min_value) / static_cast<float>(max_value - min_value) * 4.0;
	if (v < 1.0) {
		r = 0.0;
		g = 0.0;
		b = 0.5 + 0.5 * v;
	} else if (v < 2.0) {
		r = 0.0;
		g = 0.5 * (v - 1.0);
		b = 1.0;
	} else if (v < 3.0) {
		r = 0.5 * (v - 2.0);
		g = 1.0;
		b = 1.0 - 0.5 * (v - 2.0);
	} else if (v < 4.0) {
		r = 1.0;
		g = 1.0 - 0.5 * (v - 3.0);
		b = 0.0;
	} else {
		r = 1.0 - 0.5 * (v - 4.0);
		g = 0.0;
		b = 0.0;
	}
}

visualization_msgs::Marker generateVoxelMarker(Octree* octree) {
	visualization_msgs::Marker box;
	box.type = visualization_msgs::Marker::CUBE;
	box.action = visualization_msgs::Marker::ADD;
	box.pose.position.x = octree->center.x();
	box.pose.position.y = octree->center.y();
	box.pose.position.z = octree->center.z();
	box.pose.orientation.x = 0;
	box.pose.orientation.y = 0;
	box.pose.orientation.z = 0;
	box.pose.orientation.w = 1;
	box.scale.x = octree->voxelSize * 2;
	box.scale.y = octree->voxelSize * 2;
	box.scale.z = octree->voxelSize * 2;
	if (octree->planePtr->isPlane) {
		box.color.r = 1;
		box.color.g = 1;
		box.color.b = 1;
	} else {
		jetColormap(octree->depth, 0, 5, box.color.r, box.color.g, box.color.b);
	}
	box.color.a = 0.5;
	box.header.frame_id = o3d_slam::frames::mapFrame;
	box.lifetime = ros::Duration();
	box.ns = "voxels";
	box.id = id++;

	return box;
}
std::vector<visualization_msgs::Marker> traverseOctree(Octree* map) {
	std::vector<visualization_msgs::Marker> markers;
	bool hasNoLeaves = true;
	for (auto leaf : map->leaves_) {
		if (leaf != nullptr) {
			hasNoLeaves = false;
			auto subMarkers = traverseOctree(leaf);
			markers.insert(markers.end(), subMarkers.begin(), subMarkers.end());
		}
	}
	if (hasNoLeaves) {
		markers.push_back(generateVoxelMarker(map));
	}
	return markers;
}

void generateVoxelMapMarkerArray(const OctreeVoxelMap& map, const ros::Publisher& pub) {
	visualization_msgs::MarkerArray msg;
	visualization_msgs::Marker delMark;
	delMark.action = visualization_msgs::Marker::DELETEALL;
	delMark.header.frame_id = o3d_slam::frames::mapFrame;
	delMark.lifetime = ros::Duration();
	delMark.ns = "voxels";
	delMark.id = id++;

	std::vector<visualization_msgs::Marker> markers;
	for (auto el : map.voxels_) {
		auto tmp = traverseOctree(el.second);
		markers.insert(markers.end(), tmp.begin(), tmp.end());
	}
	msg.markers.push_back(delMark);
	pub.publish(msg);

	msg.markers = markers;
	pub.publish(msg);
}

void publishSubmapCoordinateAxes(const SubmapCollection& submaps, const std::string& frame_id, const ros::Time& timestamp,
																 const ros::Publisher& pub) {
	visualization_msgs::MarkerArray msg;
	int id = 0;
	msg.markers.reserve(2 * submaps.getNumSubmaps());
	for (size_t j = 0; j < submaps.getNumSubmaps(); ++j) {
		const Submap &submap = submaps.getSubmap(j);
		visualization_msgs::Marker axes, text;
		drawAxes(submap.getMapToSubmapCenter(), Eigen::Quaterniond(submap.getMapToSubmapOrigin().rotation()), 0.8, 0.08,
				&axes);
		axes.ns = "submap_" + std::to_string(j);
		axes.header.frame_id = frame_id;
		axes.header.stamp = timestamp;
		axes.id = submap.getId();
		msg.markers.push_back(axes);
		text = axes;
		text.pose.position.x += 0.3;
		text.pose.position.y += 0.3;
		text.scale.x = text.scale.y = text.scale.z = 0.4;
		text.color.r=text.color.g=text.color.b=1.0;
		text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		text.text = "(" + std::to_string(j) + ")";
		text.ns = "submap_id_" + std::to_string(j);
		text.id=id++;
		msg.markers.push_back(text);
	}
	pub.publish(msg);
}

void assembleColoredPointCloud(const SubmapCollection &submaps, open3d::geometry::PointCloud *cloud) {
	if (submaps.isEmpty()) {
		return;
	}
	std::mt19937 rndGen;
	rndGen.seed(time(NULL));
	const int nPoints = submaps.getTotalNumPoints();
	cloud->points_.reserve(nPoints);
	cloud->colors_.reserve(nPoints);
	std::uniform_int_distribution<int> rndInt(2, 12);
	for (size_t j = 0; j < submaps.getNumSubmaps(); ++j) {
		const Submap &submap = submaps.getSubmap(j);
		const auto color = Color::getColor(j % (Color::numColors_ - 2) + 2);
		const PointCloud map = submap.getMapPointCloudCopy();
		for (size_t i = 0; i < map.points_.size(); ++i) {
			cloud->points_.push_back(map.points_.at(i));
			cloud->colors_.emplace_back(Eigen::Vector3d(color.r, color.g, color.b));
		}
	}
}

void publishCloud(const open3d::geometry::PointCloud &cloud, const std::string &frame_id, const ros::Time &timestamp,
		ros::Publisher &pub) {
	if (pub.getNumSubscribers() > 0) {
		sensor_msgs::PointCloud2 msg;
		const PointCloud copy = cloud;
		open3d_conversions::open3dToRos(copy, msg, frame_id);
		msg.header.stamp = timestamp;
		pub.publish(msg);
	}
}

void publishTfTransform(const Eigen::Matrix4d &Mat, const ros::Time &time, const std::string &frame,
		const std::string &childFrame, tf2_ros::TransformBroadcaster *broadcaster) {
	geometry_msgs::TransformStamped transformStamped = o3d_slam::toRos(Mat, time, frame, childFrame);
	broadcaster->sendTransform(transformStamped);
}

bool lookupTransform(const std::string &target_frame, const std::string &source_frame, const ros::Time &time,
		const tf2_ros::Buffer &tfBuffer, Eigen::Isometry3d *transform) {
	geometry_msgs::TransformStamped transformStamped;
	try {
		transformStamped = tfBuffer.lookupTransform(target_frame, source_frame, time);
	} catch (tf2::TransformException &ex) {
		ROS_WARN("caught exception while looking up the tf: %s", ex.what());
		*transform = Eigen::Isometry3d::Identity();
		return false;
	}
	*transform = tf2::transformToEigen(transformStamped);
	return true;
}

geometry_msgs::Pose getPose(const Eigen::MatrixXd &T) {
	geometry_msgs::Pose pose;

	// Fill pose
	Eigen::Affine3d eigenTr;
	eigenTr.matrix() = T;
	tf::poseEigenToMsg(eigenTr, pose);

	return pose;
}

geometry_msgs::TransformStamped toRos(const Eigen::Matrix4d &Mat, const ros::Time &time, const std::string &frame,
		const std::string &childFrame) {

	geometry_msgs::TransformStamped transformStamped;
	transformStamped.header.stamp = time;
	transformStamped.header.frame_id = frame;
	transformStamped.child_frame_id = childFrame;
	const auto pose = getPose(Mat);
	transformStamped.transform.translation.x = pose.position.x;
	transformStamped.transform.translation.y = pose.position.y;
	transformStamped.transform.translation.z = pose.position.z;
	transformStamped.transform.rotation = pose.orientation;
	return transformStamped;
}

geometry_msgs::Point createPoint(double x, double y, double z) {
	geometry_msgs::Point p;
	p.x = x;
	p.y = y;
	p.z = z;
	return p;
}

void drawAxes(const Eigen::Vector3d &p, const Eigen::Quaterniond &q, double scale, double line_width,
		visualization_msgs::Marker *marker) {
	marker->colors.resize(6);
	marker->points.resize(6);
	marker->points[0] = createPoint(0, 0, 0);
	marker->points[1] = createPoint(1 * scale, 0, 0);
	marker->points[2] = createPoint(0, 0, 0);
	marker->points[3] = createPoint(0, 1 * scale, 0);
	marker->points[4] = createPoint(0, 0, 0);
	marker->points[5] = createPoint(0, 0, 1 * scale);

	marker->color = Color::Black();
	marker->colors[0] = Color::Red();
	marker->colors[1] = Color::Red();
	marker->colors[2] = Color::Green();
	marker->colors[3] = Color::Green();
	marker->colors[4] = Color::Blue();
	marker->colors[5] = Color::Blue();

	marker->scale.x = line_width;  // rest is unused
	marker->type = visualization_msgs::Marker::LINE_LIST;
	marker->action = visualization_msgs::Marker::ADD;

	tf::pointEigenToMsg(p, marker->pose.position);
	tf::quaternionEigenToMsg(q, marker->pose.orientation);
}


ros::Time toRos(Time time) {
	int64_t uts_timestamp = toUniversal(time);
	int64_t ns_since_unix_epoch = (uts_timestamp - kUtsEpochOffsetFromUnixEpochInSeconds * 10000000ll) * 100ll;
	::ros::Time ros_time;
	if (ns_since_unix_epoch < 0) {
		std::cerr << "ERROR: nanoseconds since unix epoch is: " << ns_since_unix_epoch
				<< " which is impossible!!!! \n";
		std::cerr << "       ROS time will throw you an exception fo sho!!!! \n";
		std::cerr << "       Are you playing the rosbag with --clock??? \n";
		std::cerr << "       If yes, did you set use_sim_time to true ??? \n";
		std::cout << "Universal time: " << uts_timestamp << std::endl;
	}
	ros_time.fromNSec(ns_since_unix_epoch);
	return ros_time;
}

Time fromRos(const ::ros::Time &time) {
	// The epoch of the ICU Universal Time Scale is "0001-01-01 00:00:00.0 +0000",
	// exactly 719162 days before the Unix epoch.
	return fromUniversal(
			(time.sec + kUtsEpochOffsetFromUnixEpochInSeconds) * 10000000ll + (time.nsec + 50) / 100); // + 50 to get the rounding correct.
}

} /* namespace o3d_slam */
