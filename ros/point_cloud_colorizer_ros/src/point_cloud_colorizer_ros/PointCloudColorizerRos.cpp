//! Include the header file.
#include "point_cloud_colorizer_ros/PointCloudColorizerRos.hpp"

// message logger
#include <message_logger/message_logger.hpp>

// cv bridge
#include <cv_bridge/cv_bridge.h>

namespace point_cloud_colorizer_ros {

PointCloudColorizerRos::PointCloudColorizerRos(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle), tfListener_(tfBuffer_), imTransport_(nodeHandle) {
  // Read parameters from ROS server.
  if (!readParameters()) {
    ros::requestShutdown();
  }

  // Setup publishers.
  setUpPublishers();

  // Setup subscribers.
  setUpSubscribers();

  MELO_INFO_STREAM("Initialized node for colorizing the point cloud.");
}

void PointCloudColorizerRos::setUpSubscribers() {
  // Input point cloud subscriber.
  pointCloudSub_ = nodeHandle_.subscribe(parameters_.inputPointCloudTopic_, 10, &PointCloudColorizerRos::inputPointCloudCb, this);

  // Setup Cameras.
  for (size_t id = 1; id <= parameters_.numOfSupportedCameras_; id++) {
    setUpCameraSubscribers(id, "color_image_" + std::to_string(id));
  }
}

void PointCloudColorizerRos::setUpPublishers() {
  // Setup colorized point cloud publisher
  colorCloudPublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>(parameters_.outputPointCloudTopic_, 10, false);

  // Camera images with overlaid point cloud and depth images. Iterate over available number of cameras.
  for (size_t id = 1; id <= parameters_.numOfSupportedCameras_; id++) {
    imagePublishers_[id] = imTransport_.advertise("point_cloud_overlaid_camera_" + std::to_string(id), 1);
    depthImagePublishers_[id] = imTransport_.advertise("depth_image_camera_" + std::to_string(id), 1);
  }
}

void PointCloudColorizerRos::setUpCameraSubscribers(unsigned int id, const std::string& topicKey) {
  // Set up camera subscriber.
  std::string imageTopicName{""};

  if (!nodeHandle_.param<std::string>("subscribers/" + topicKey + "/topic", imageTopicName, "")) {
    MELO_WARN_STREAM("Cannot find the image topic for camera " << id);
    return;
  }

  // Create camera image buffer.
  cameraIdToImageBufferMap_[id].imageCache_.setCacheSize(parameters_.imgCacheSize_);

  cameraIdToImageBufferMap_[id].imageSub_ =
      nodeHandle_.subscribe<sensor_msgs::Image>(imageTopicName, 10, boost::bind(&PointCloudColorizerRos::inputImageCb, this, _1, id));
}

void PointCloudColorizerRos::inputImageCb(const sensor_msgs::ImageConstPtr& image, unsigned int id) {
  MELO_DEBUG_STREAM("Received image from camera " << id << " Encoding: " << image->encoding);

  /*
  MELO_WARN_STREAM("PC STAMP AT HAND:  " << currentLidarCloud_->header.stamp);
  MELO_WARN_STREAM("received STAMP:  " << image->header.stamp);
  MELO_WARN_STREAM("Stamp at callback " << (currentLidarCloud_->header.stamp - image->header.stamp).toSec());
  */

  if ((image->encoding != "bgr8") && (image->encoding != "bgra8")) {
    MELO_WARN_STREAM("Wrong encoding of image not processing. Encoding:  " << image->encoding);
    return;
  }

  cameraIdToImageBufferMap_[id].imageCache_.add(image);
  isCameraAlreadyProcessed_[id] = false;
}

bool PointCloudColorizerRos::readCameraCalibrationMatrix() {
  readCameraInfoTopics();
  MELO_DEBUG_STREAM("There are " << availableCameraInfosVec_.size() << " camera(s) available.");
  return !availableCameraInfosVec_.empty();
}

void PointCloudColorizerRos::readCameraInfoTopics() {
  // Set the camera matrix counter to 0. All matrices should be continuously streamed anyway.
  readCameraMatrixCounter_ = 0;
  for (auto& cameraIdToImageBufferMapIter : cameraIdToImageBufferMap_) {
    std::string cameraInfoTopic{""};
    nodeHandle_.param<std::string>("subscribers/color_image_" + std::to_string(cameraIdToImageBufferMapIter.first) + "/camera_info_topic",
                                   cameraInfoTopic, "");

    boost::shared_ptr<const sensor_msgs::CameraInfo> cameraInfoMsgPtr =
        ros::topic::waitForMessage<sensor_msgs::CameraInfo>(cameraInfoTopic, ros::Duration(parameters_.findCameraInfoTimeOut_));
    if (!cameraInfoMsgPtr) {
      MELO_WARN_STREAM("No camera info topic for Camera id: " << cameraIdToImageBufferMapIter.first);
    } else {
      availableCameraInfosVec_.push_back(cameraIdToImageBufferMapIter.first);

      colorizer_.cameraParameters_[cameraIdToImageBufferMapIter.first].intrinsicCameraMatrix_ =
          Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(cameraInfoMsgPtr->K.data());
      MELO_INFO_STREAM("Camera id: " << cameraIdToImageBufferMapIter.first << ": camera matrix: \n"
                                     << colorizer_.cameraParameters_[cameraIdToImageBufferMapIter.first].intrinsicCameraMatrix_);

      // Successfully read the camera matrix, increase the counter.
      readCameraMatrixCounter_++;
    }
  }
}

void PointCloudColorizerRos::inputPointCloudCb(const sensor_msgs::PointCloud2::ConstPtr& lidarCloud) {
  const ros::WallTime startTime{ros::WallTime::now()};
  currentLidarCloud_ = lidarCloud;

  if (!readCalibrationMatrix_) {
    availableCameraInfosVec_.clear();
    if (!readCameraCalibrationMatrix()) {
      MELO_WARN_STREAM("Point cloud colorizer is idle, no camera available.");
      return;
    }

    if (readCameraMatrixCounter_ == parameters_.numOfSupportedCameras_) {
      // We won't check again whether there are camera matrices.
      readCalibrationMatrix_ = true;
      MELO_INFO_STREAM("All expected Camera Information Matrices were found. Won't look for more. Number: "
                       << readCameraMatrixCounter_ << " / " << parameters_.numOfSupportedCameras_);
    }
  }

  availableCamerasVec_.clear();
  readImageAndTransformation(availableCameraInfosVec_);

  MELO_DEBUG_STREAM("Number of cameras for colorization is: " << availableCamerasVec_.size());

  if (availableCamerasVec_.empty()) {
    MELO_DEBUG_STREAM("No available cameras for colorization. Colorization won't be executed.");
    return;
  }

  // Construct a pointer
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl_cloud_ptr->reserve(lidarCloud->width * lidarCloud->height);

  pcl::fromROSMsg(*lidarCloud, *pcl_cloud_ptr);
  PointCloud pointCloudData;
  pcl::copyPointCloud(*pcl_cloud_ptr, pointCloudData);

  // Main colorization and depth generation function
  PointCloud colorizedPoints = colorizer_.colorizePoints(pointCloudData, availableCamerasVec_);

  // Publish the colorized point cloud
  sensor_msgs::PointCloud2 pointsInPlaneMsg;
  pcl::toROSMsg(colorizedPoints, pointsInPlaneMsg);
  pointsInPlaneMsg.header = lidarCloud->header;
  colorCloudPublisher_.publish(pointsInPlaneMsg);

  const ros::WallDuration processTime{ros::WallTime::now() - startTime};
  MELO_DEBUG_STREAM("Processing time: " << processTime);

  MELO_DEBUG("Publishing depth and overlaid images.");

  if (colorizer_.generateOverlaidImages_) {
    for (size_t id = 1; id <= parameters_.numOfSupportedCameras_; id++) {
      if (imagePublishers_[id].getNumSubscribers() > 0u) {
        const cv::Mat image{colorizer_.cameraParameters_[id].cameraImgWithPointCloudOverlaid_.clone()};

        if (image.empty()) {
          MELO_WARN_STREAM("Image empty not publishing id: " << id);
          continue;
        }

        sensor_msgs::ImagePtr imageMsg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGRA8, image).toImageMsg();
        imageMsg->header = lidarCloud->header;
        imagePublishers_[id].publish(imageMsg);
      }
    }
  }

  if (!colorizer_.generateDepthImages_) {
    return;
  }

  for (size_t id = 1; id <= parameters_.numOfSupportedCameras_; id++) {
    if (depthImagePublishers_[id].getNumSubscribers() > 0u) {
      const cv::Mat depthImage{colorizer_.cameraParameters_[id].depthImgFromPoints_.clone()};
      sensor_msgs::ImagePtr depthImageMsg = cv_bridge::CvImage(std_msgs::Header(), "32FC1", depthImage).toImageMsg();

      depthImageMsg->header = imageHeaderBuffer_[id];
      depthImagePublishers_[id].publish(depthImageMsg);
    }
  }
}

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

  success &= nodeHandle_.param<bool>("use_helper_frame", useHelperFrame_, 1);

  return success;
}

void PointCloudColorizerRos::readImageAndTransformation(const std::vector<unsigned int>& availableCameraInfosVec) {
  availableCamerasVec_ = availableCameraInfosVec;
  const ros::Time lidarCloudTimeStamp = currentLidarCloud_->header.stamp;
  auto vecIterator = availableCamerasVec_.begin();
  while (vecIterator != availableCamerasVec_.end()) {
    if (!syncronizeImage(*vecIterator)) {
      MELO_DEBUG_STREAM("Cannot obtain latest image from Camera " << *vecIterator << " for input point cloud.");
      vecIterator = availableCamerasVec_.erase(vecIterator);
      continue;
    }
    // Store the header
    imageHeaderBuffer_[*vecIterator] = tmpLatestImage_->header;

    // Set the image information
    colorizer_.cameraParameters_[*vecIterator].imgHeight_ = tmpLatestImage_->height;
    colorizer_.cameraParameters_[*vecIterator].imgWidth_ = tmpLatestImage_->width;

    try {
      // RGB Vs BGR doesnt make any difference in the code. But for the user nodes it does.
      auto cvImgPtr_ = cv_bridge::toCvCopy(tmpLatestImage_, sensor_msgs::image_encodings::BGRA8);

      /*
      for (int i = 0; i < cvImgPtr_->image.rows; i++) {
        for (int j = 0; j < cvImgPtr_->image.cols; j++) {
          cv::Vec4b& pixel = cvImgPtr_->image.at<cv::Vec4b>(i, j);
          MELO_ERROR_STREAM("pixel[3]: " << pixel[3]);
          MELO_ERROR_STREAM("int casted pixel[3]: " << static_cast<int>(pixel[3]));
        }
      }
      */

      colorizer_.cameraParameters_[*vecIterator].cvImg_ = cvImgPtr_->image;
    } catch (cv_bridge::Exception& e) {
      MELO_ERROR_STREAM("cv_bridge exception: " << e.what());
      vecIterator = availableCamerasVec_.erase(vecIterator);
      continue;
    }

    std::string targetFrame = (useHelperFrame_) ? (tmpLatestImage_->header.frame_id + "_helper") : (tmpLatestImage_->header.frame_id);

    //! Find the transformation between currentLidarCloud_ and camera.
    if (!tfBuffer_.canTransform(targetFrame, tmpLatestImage_->header.stamp, currentLidarCloud_->header.frame_id, lidarCloudTimeStamp,
                                parameters_.fixedFrame_, ros::Duration(parameters_.findTfTimeOut_))) {
      MELO_WARN_STREAM("Timeout for finding the transformation between current input point cloud ["
                       << currentLidarCloud_->header.frame_id << "] and Camera " << *vecIterator << " frame ["
                       << tmpLatestImage_->header.frame_id << "].");
      vecIterator = availableCamerasVec_.erase(vecIterator);
      continue;
    } else {
      // Currently on Cerberus there is a `camX_sensor_frame_helper` frame which contains the proper transformation.
      // This was different in the first mission.

      auto lidarToCameraTrans =
          tfBuffer_.lookupTransform(targetFrame, tmpLatestImage_->header.stamp, currentLidarCloud_->header.frame_id, lidarCloudTimeStamp,
                                    parameters_.fixedFrame_, ros::Duration(parameters_.findTfTimeOut_));

      // Set the transformations to their respective cameras.
      colorizer_.cameraParameters_[*vecIterator].pclTransform_ = lidarToCameraTrans;
    }

    vecIterator++;
  }
}

bool PointCloudColorizerRos::syncronizeImage(int id) {
  //! Get the timestamp for current point cloud.
  const ros::Time lidarCloudTimeStamp = currentLidarCloud_->header.stamp;

  // Check image is used.
  bool validFlag{false};

  if (isCameraAlreadyProcessed_[id]) {
    MELO_DEBUG_STREAM("Already processed camera " << id);
    return validFlag;
  }

  // An ugly trick, should be functionized.
  for (size_t i = 0; i < 1; i++) {
    //! Get the latest image before the timestamp of current point cloud.
    tmpLatestImage_ = cameraIdToImageBufferMap_[id].imageCache_.getElemBeforeTime(lidarCloudTimeStamp);
    //! If we cannot find it, we should wait.
    if (!tmpLatestImage_) {
      MELO_DEBUG_STREAM("Cannot obtain latest previous image from Camera " << id << " for input point cloud. ");
      continue;
    }

    const auto timeDifference = (lidarCloudTimeStamp - tmpLatestImage_->header.stamp).toSec();
    MELO_DEBUG_STREAM("The timestamp difference between latest previous image from Camera "
                      << id << " and input point cloud: " << timeDifference);
    if (std::fabs(timeDifference) > parameters_.allowedCameraPointcloudTimeDiff_) {
      MELO_DEBUG_STREAM("The timestamp difference between latest previous image from Camera " << id
                                                                                              << " and input point cloud is too big.");
      continue;
    }
    //! Set validFlag to be true if all constraints are passed.
    validFlag = true;
    // We won't look for syncronization of this camera until a new image arrives for this camera.
    isCameraAlreadyProcessed_[id] = true;
    break;
  }

  //! If we cannot find the latest previous image for current point cloud, we will try to look for an upcoming image for a while.
  if (!validFlag) {
    MELO_DEBUG_STREAM("Checking for Latter images");
    tmpLatestImage_ = cameraIdToImageBufferMap_[id].imageCache_.getElemAfterTime(lidarCloudTimeStamp);
    if (!tmpLatestImage_) {
      MELO_DEBUG_STREAM("Cannot obtain latest latter image from Camera " << id << " for input point cloud. ");
      return validFlag;
    }
    const auto timeDifference = (lidarCloudTimeStamp - tmpLatestImage_->header.stamp).toSec();
    MELO_DEBUG_STREAM("The timestamp difference between latest latter image from Camera " << id
                                                                                          << " and input point cloud: " << timeDifference);
    if (std::fabs(timeDifference) > parameters_.allowedCameraPointcloudTimeDiff_) {
      MELO_DEBUG_STREAM("The timestamp difference between latest latter image from Camera " << id << " and input point cloud is too big.");
      return validFlag;
    }
  }

  // We won't look for syncronization of this camera until a new image arrives for this camera.
  isCameraAlreadyProcessed_[id] = true;
  return true;
}
}  // namespace point_cloud_colorizer_ros
