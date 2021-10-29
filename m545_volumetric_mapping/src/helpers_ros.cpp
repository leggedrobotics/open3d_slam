/*
 * helpers_ros.cpp
 *
 *  Created on: Oct 20, 2021
 *      Author: jelavice
 */

#include "m545_volumetric_mapping/helpers_ros.hpp"
#include "m545_volumetric_mapping/Submap.hpp"
#include <random>
// ros stuff
#include "open3d_conversions/open3d_conversions.h"
#include <eigen_conversions/eigen_msg.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <nav_msgs/Odometry.h>
#include <m545_volumetric_mapping_msgs/PolygonMesh.h>

namespace m545_mapping {

namespace {
template<typename T, typename Limits>
void clamp(T *val, Limits lo, Limits hi) {
	if (*val > hi) {
		*val = hi;
		return;
	}
	if (*val < lo) {
		*val = lo;
		return;
	}
}
} // namespace

Color::Color() :
		std_msgs::ColorRGBA() {
}
Color::Color(double red, double green, double blue) :
		Color(red, green, blue, 1.0) {
}
Color::Color(double red, double green, double blue, double alpha) :
		Color() {
	r = red;
	g = green;
	b = blue;
	a = alpha;
}

Color Color::operator*(double scalar) const {
	Color ret = *this;
	ret.r *= scalar;
	ret.g *= scalar;
	ret.b *= scalar;
	clamp(&ret.r, 0.0, 1.0);
	clamp(&ret.g, 0.0, 1.0);
	clamp(&ret.b, 0.0, 1.0);
	return ret;
}
Color operator*(double scalar, const Color &c) {
	return c * scalar;
}




void assembleColoredPointCloud(const SubmapCollection &submaps, open3d::geometry::PointCloud *cloud){
	if(submaps.isEmpty()){
		return;
	}
	std::mt19937 rndGen;
	rndGen.seed(time(NULL));
	const int nSubmaps = submaps.getSubmaps().size();
	const  int nPoints = submaps.getTotalNumPoints();
	cloud->points_.reserve(nPoints);
	cloud->colors_.reserve(nPoints);
	std::uniform_int_distribution<int> rndInt(2,12);
	for(size_t j =0;j < submaps.getSubmaps().size(); ++j){
		const auto &submap = submaps.getSubmaps().at(j);
		const auto color = Color::getColor(j % (Color::numColors_-2) +2);
		for(size_t i =0;i < submap.getMap().points_.size(); ++i){
			cloud->points_.push_back(submap.getMap().points_.at(i));
			cloud->colors_.emplace_back(Eigen::Vector3d(color.r,color.g,color.b));
		}
	}
}

void publishMesh(const open3d::geometry::MeshBase &mesh, const std::string &frame_id, const ros::Time &timestamp,
		ros::Publisher &pub) {
	if (pub.getNumSubscribers() > 0) {
		m545_volumetric_mapping_msgs::PolygonMesh meshMsg;
		open3d_conversions::open3dToRos(mesh, frame_id, meshMsg);
		meshMsg.header.frame_id = frame_id;
		meshMsg.header.stamp = timestamp;
		pub.publish(meshMsg);
	}
}

void publishCloud(const open3d::geometry::PointCloud &cloud, const std::string &frame_id, const ros::Time &timestamp,
		ros::Publisher &pub) {
	if (pub.getNumSubscribers() > 0) {
		sensor_msgs::PointCloud2 msg;
		open3d_conversions::open3dToRos(cloud, msg, frame_id);
		msg.header.stamp = timestamp;
		pub.publish(msg);
	}
}

void publishTfTransform(const Eigen::Matrix4d &Mat, const ros::Time &time, const std::string &frame,
		const std::string &childFrame, tf2_ros::TransformBroadcaster *broadcaster) {
	geometry_msgs::TransformStamped transformStamped = m545_mapping::toRos(Mat, time, frame, childFrame);
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

const Color Color::getColor (int colorCode){

	switch (colorCode){
	case 0: {
		return White();
	}
	case 1: {
		return Black();
	}
	case 2: {
		return Gray();
	}
	case 3: {
		return Red();
	}
	case 4: {
		return Green();
	}
	case 5: {
		return Blue();
	}
	case 6: {
		return Yellow();
	}
	case 7: {
		return Orange();
	}
	case 8: {
		return Purple();
	}
	case 9: {
		return Chartreuse();
	}
	case 10: {
		return Teal();
	}
	case 11: {
		return Pink();
	}
	case 12: {
		return Magenta();
	}
	default:
		throw std::runtime_error("unknown color code");
	}

}

} /* namespace m545_mapping */
