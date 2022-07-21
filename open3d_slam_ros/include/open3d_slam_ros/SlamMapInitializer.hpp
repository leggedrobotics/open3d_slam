/*
 * OnlineDataProcessorRos.hpp
 *
 *  Created on: Apr 21, 2022
 *      Author: jelavice
 */

#pragma once

#include <memory>
#include <atomic>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "open3d_slam/SlamWrapper.hpp"
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

namespace o3d_slam {

class SlamMapInitializer{

public:
	SlamMapInitializer(std::shared_ptr<SlamWrapper> slamPtr, ros::NodeHandlePtr nh);
	~SlamMapInitializer() = default;

	void initialize();

private:
	void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
	void initInterectiveMarker();
	void interactiveMarkerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& msg);
	visualization_msgs::InteractiveMarker createInteractiveMarker() const;
	
  interactive_markers::MenuHandler menuHandler_;
  interactive_markers::InteractiveMarkerServer server_;
	std::shared_ptr<SlamWrapper> slamPtr_;
	ros::Subscriber cloudSubscriber_;
	std::atomic_bool initialized_;
	std::string frameId_;
	std::string meshResourcePath_;
	std::string pcdFilePath_;
	ros::NodeHandlePtr nh_;

};

} // namespace o3d_slam
