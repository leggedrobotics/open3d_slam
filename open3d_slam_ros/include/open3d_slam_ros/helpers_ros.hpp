/*
 * helpers_ros.hpp
 *
 *  Created on: Oct 20, 2021
 *      Author: jelavice
 */

#pragma once
#include <open3d/geometry/PointCloud.h>
#include <open3d/geometry/MeshBase.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/publisher.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/MarkerArray.h>


namespace o3d_slam {

class SubmapCollection;

class Color : public std_msgs::ColorRGBA {
 public:
  Color();
  Color(double red, double green, double blue);
  Color(double red, double green, double blue, double alpha);
  Color operator*(double scalar) const;

  static const Color White() { return Color(1.0, 1.0, 1.0); }
  static const Color Black() { return Color(0.0, 0.0, 0.0); }
  static const Color Gray() { return Color(0.5, 0.5, 0.5); }
  static const Color Red() { return Color(1.0, 0.0, 0.0); }
  static const Color Green() { return Color(0.0, 1.0, 0.0); }
  static const Color Blue() { return Color(0.0, 0.0, 1.0); }
  static const Color Yellow() { return Color(1.0, 1.0, 0.0); }
  static const Color Orange() { return Color(1.0, 0.5, 0.0); }
  static const Color Purple() { return Color(0.5, 0.0, 1.0); }
  static const Color Chartreuse() { return Color(0.5, 1.0, 0.0); }
  static const Color Teal() { return Color(0.0, 1.0, 1.0); }
  static const Color Pink() { return Color(1.0, 0.0, 0.5); }
  static const Color Magenta() { return Color(0.78, 0.0, 0.9); }
  static const int numColors_ = 13;
  static const Color getColor (int colorCode);
};

void publishSubmapCoordinateAxes(const SubmapCollection &submaps, const std::string &frame_id,
		const ros::Time &timestamp, const ros::Publisher &pub);
geometry_msgs::Point createPoint(double x, double y, double z);
void drawAxes(const Eigen::Vector3d& p, const Eigen::Quaterniond& q, double scale, double line_width, visualization_msgs::Marker* marker);

void assembleColoredPointCloud(const SubmapCollection &submaps, open3d::geometry::PointCloud *cloud);

void publishCloud(const open3d::geometry::PointCloud &cloud, const std::string &frame_id, const ros::Time &timestamp,ros::Publisher &pub);

geometry_msgs::Pose getPose(const Eigen::MatrixXd &T);

geometry_msgs::TransformStamped toRos(const Eigen::Matrix4d &Mat, const ros::Time &time, const std::string &frame,
		const std::string &childFrame);

void publishTfTransform(const Eigen::Matrix4d &Mat, const ros::Time &time, const std::string &frame,
		const std::string &childFrame, tf2_ros::TransformBroadcaster *broadcaster);
bool lookupTransform(const std::string &target_frame, const std::string &source_frame, const ros::Time &time,const tf2_ros::Buffer &tfBuffer,
		Eigen::Isometry3d *transform);

} /* namespace o3d_slam */
