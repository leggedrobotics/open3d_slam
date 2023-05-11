#pragma once

#include <string>

namespace point_cloud_colorizer_ros {

struct Parameters {
  //! Time allowed to find and read camera_info topics. Unit: second.
  float findCameraInfoTimeOut_{5.0f};

  //! The cache size to hold the front and rear RGB images.
  unsigned int imgCacheSize_{100u};

  //! Time allowed to find the transformation between input point cloud frame and camera frame.
  float findTfTimeOut_{1.0f};

  //! The name of the fixed frame.
  std::string fixedFrame_{"odom"};

  //! Time difference allowed between the timestamps of input point cloud and the latest image.
  float allowedCameraPointcloudTimeDiff_{0.5f};

  //! Number of supported cameras.
  unsigned int numOfSupportedCameras_{2u};

  //! Subscribed Point Cloud Topic
  std::string inputPointCloudTopic_{"/point_cloud_filter/lidar/point_cloud_filtered"};

  //! Output colorized point cloud topic
  std::string outputPointCloudTopic_{"colorized_point_cloud"};

  //! Use helper frame
  bool useHelperFrame_{false};

  // Image transport type
  std::string imageTransportType_;
};

}  // namespace point_cloud_colorizer_ros