#include <ros/ros.h>
#include "realtime_meshing_ros/RealtimeMeshingRos.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "realtime_meshing_node");
  ros::NodeHandlePtr nh(new ros::NodeHandle("~"));
  ROS_INFO("Starting up...");
  RealtimeMeshingRos mesher(nh);
  ros::spin();

  return 0;
}
