#pragma once

// ros
#include <ros/ros.h>

// message filters
#include <message_filters/cache.h>

// Message Types
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>

// image transport
#include <image_transport/image_transport.h>

// tf2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// PCL Types.
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// Colorizer backend.
#include "point_cloud_colorizer/PointCloudColorizer.hpp"
#include "point_cloud_colorizer_ros/Parameters.hpp"

namespace point_cloud_colorizer_ros {

// Backend colorizer.
using PointCloudColorizer = point_cloud_colorizer::PointCloudColorizer;

struct ImageBuffer {
  image_transport::Subscriber imageSub_;
  message_filters::Cache<sensor_msgs::Image> imageCache_;
};

class PointCloudColorizerRos {
 public:
  //! Explicit constructor
  explicit PointCloudColorizerRos(ros::NodeHandle& nodeHandle);

 private:
  //! Set up the ros publishers
  void setUpPublishers();

  //! Set up the ros sub
  void setUpSubscribers();

  //! Read parameters.
  bool readParameters();

  //! Set the camera subsrucbers based on camera id and name.
  void setUpCameraSubscribers(unsigned int id, const std::string& topicKey);

  //! Read and report camera info topics.
  bool readCameraCalibrationMatrix();

  //! Check and read the camera info topics
  void readCameraInfoTopics();

  //! Multiple image callback.
  void inputImageCallback(const sensor_msgs::ImageConstPtr& image, unsigned int id);

  //! Main callback for point cloud
  void inputPointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& lidarCloud);

  //! Check the received images and based on their availability find the transform from lidar to cameras.
  void readImageAndTransformation(const std::vector<unsigned int>& availableCameraInfosVec);

  //! A logical checker to identify the closest image to the recently arrived.
  bool synchronizeImage(int id);

  //! Rosnode handle
  ros::NodeHandle nodeHandle_;

  //! TF
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;

  //! Publishers.
  ros::Publisher colorCloudPublisher_;
  image_transport::ImageTransport imTransport_;
  std::map<unsigned int, image_transport::Publisher> imagePublishers_;
  std::map<unsigned int, image_transport::Publisher> depthImagePublishers_;

  //! Subscribers.
  ros::Subscriber pointCloudSub_;

  //! Whether the camera calibration matrix has been received.
  bool readCalibrationMatrix_{false};
  std::vector<unsigned int> availableCameraInfosVec_;

  //! Get an colorizer backend object.
  PointCloudColorizer colorizer_;

  //! The read camera info counter. To prevent reading again.
  unsigned int readCameraMatrixCounter_{0u};

  //! Temporary storage of latest point cloud and images.
  sensor_msgs::PointCloud2ConstPtr currentLidarCloud_;
  boost::shared_ptr<const sensor_msgs::Image> tmpLatestImage_;

  //! Buffers
  std::map<unsigned int, ImageBuffer> cameraIdToImageBufferMap_;
  std::vector<unsigned int> availableCamerasVec_;
  std::map<unsigned int, std_msgs::Header> imageHeaderBuffer_;
  std::map<unsigned int, bool> isCameraAlreadyProcessed_;

  //! Parameter for the node to run.
  Parameters parameters_;
};
}  // namespace point_cloud_colorizer_ros
