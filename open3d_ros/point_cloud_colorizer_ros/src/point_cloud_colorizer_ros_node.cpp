
#include "point_cloud_colorizer_ros/PointCloudColorizerRos.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "point_cloud_colorizer_ros");
  ros::NodeHandle nodeHandle("~");
  point_cloud_colorizer_ros::PointCloudColorizerRos pointCloudColorizer(nodeHandle);
  // Single threaded spinner.
  ros::spin();
  return EXIT_SUCCESS;
}