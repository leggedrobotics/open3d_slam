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

// Colorizer backend.
#include <point_cloud_colorizer/PointCloudColorizer.hpp>

// PCL Types.
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace point_cloud_colorizer_ros {

// Backend colorizer.
using PointCloudColorizer = point_cloud_colorizer::PointCloudColorizer;

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
};

struct ImageBuffer {
  ros::Subscriber imageSub_;
  message_filters::Cache<sensor_msgs::Image> imageCache_;
};

class PointCloudColorizerRos {
 public:
  // For ROS transport.
  using ImageTransport = image_transport::ImageTransport;
  using ImagePublisher = image_transport::Publisher;

  // Set common PCL types
  using Point = pcl::PointXYZRGBL;
  using PointCloud = pcl::PointCloud<Point>;
  using PointCloudPtr = PointCloud::Ptr;
  using PointCloudConstPtr = PointCloud::ConstPtr;

  //! Explicit constructor
  explicit PointCloudColorizerRos(ros::NodeHandle& nodeHandle);

 private:
  //! Set up the ros publushers
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
  void inputImageCb(const sensor_msgs::ImageConstPtr& image, unsigned int id);

  //! Main callback for point cloud
  void inputPointCloudCb(const sensor_msgs::PointCloud2ConstPtr& lidarCloud);

  //! Check the received images and based on their availability find the tranform from lidar to cameras.
  void readImageAndTransformation(const std::vector<unsigned int>& availableCameraInfosVec);

  //! A logical checker to identify the closest image to the recently arrived.
  bool syncronizeImage(int id);

  //! Rosnode handle
  ros::NodeHandle nodeHandle_;

  //! TF
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;

  //! Publishers.
  ros::Publisher colorCloudPublisher_;
  ImageTransport imTransport_;
  std::map<unsigned int, ImagePublisher> imagePublishers_;
  std::map<unsigned int, ImagePublisher> depthImagePublishers_;

  //! Subscribers.
  ros::Subscriber pointCloudSub_;

  //! Whether the camera calibration matrix has been received.
  bool readCalibrationMatrix_{false};
  std::vector<unsigned int> availableCameraInfosVec_;

  //! Get an colorizer backend object.
  PointCloudColorizer colorizer_;

  //! The read camera info counter. To prevent reading again.
  unsigned int readCameraMatrixCounter_{0u};

  //! Use helper frame
  bool useHelperFrame_{false};

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
