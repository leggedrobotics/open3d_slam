#include "point_cloud_colorizer_ros/PointCloudColorizerRos.hpp"

namespace point_cloud_colorizer_ros {

bool PointCloudColorizerRos::readParameters() {
  // Read parameters
  bool success = true;
  float maxColorizationDepth{0.0f};
  success &= nodeHandle_.param<float>("visualization/max_colorization_depth", maxColorizationDepth, 40.0f);

  // Set max distance to backend.
  colorizer_.maxColorizationDepth_ = std::move(maxColorizationDepth);

  int bloatFactor{0};
  success &= nodeHandle_.param<int>("visualization/bloat_factor", bloatFactor, 4);
  colorizer_.bloatFactor_ = std::move(bloatFactor);

  bool generateOverlaidImages{false};
  success &= nodeHandle_.param<bool>("generate_overlaid_images", generateOverlaidImages, false);
  colorizer_.generateOverlaidImages_ = std::move(generateOverlaidImages);

  if (!generateOverlaidImages) {
    MELO_WARN("Overlaid images wont be published.");
  }

  bool generateDepthImages{false};
  success &= nodeHandle_.param<bool>("generate_depth_images", generateDepthImages, false);
  colorizer_.generateDepthImages_ = std::move(generateDepthImages);

  success &= nodeHandle_.param<float>("time/allowed_camera_info_find_time", parameters_.findCameraInfoTimeOut_, 4.0f);
  success &= nodeHandle_.param<float>("time/allowed_tf_find_time", parameters_.findTfTimeOut_, 1.0f);

  success &= nodeHandle_.param<float>("time/allowed_camera_point_cloud_time_diff", parameters_.allowedCameraPointcloudTimeDiff_, 0.5f);

  int imgCacheSize_{0};
  success &= nodeHandle_.param<int>("image_frame_cache_size", imgCacheSize_, 100);
  parameters_.imgCacheSize_ = static_cast<unsigned int>(imgCacheSize_);

  success &= nodeHandle_.param<std::string>("fixed_frame", parameters_.fixedFrame_, "odom");
  success &= nodeHandle_.param<std::string>("input_point_cloud_topic", parameters_.inputPointCloudTopic_,
                                            "/point_cloud_filter/lidar/point_cloud_filtered");

  success &= nodeHandle_.param<std::string>("output_point_cloud_topic", parameters_.outputPointCloudTopic_, "colorized_point_cloud");

  int numOfSupportedCameras_{0};
  success &= nodeHandle_.param<int>("num_of_supported_cameras", numOfSupportedCameras_, 1);
  parameters_.numOfSupportedCameras_ = static_cast<unsigned int>(numOfSupportedCameras_);

  success &= nodeHandle_.param<bool>("use_helper_frame", parameters_.useHelperFrame_, 1);

  success &= nodeHandle_.param<std::string>("image_transport", parameters_.imageTransportType_, "base_link");

  return success;
}

}  // namespace point_cloud_colorizer_ros