//! Include the header file.
#include "point_cloud_colorizer_ros/PointCloudColorizerRos.hpp"

// message logger
#include <message_logger/message_logger.hpp>

// cv bridge
#include <cv_bridge/cv_bridge.h>

#include <open3d_conversions/open3d_conversions.h>

namespace point_cloud_colorizer_ros {

PointCloudColorizerRos::PointCloudColorizerRos(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle), tfListener_(tfBuffer_), imTransport_(nodeHandle) {
  // Read parameters from ROS server.
  if (!readParameters()) {
    ros::requestShutdown();
  }

  // Image Transport
  imTransport_ = image_transport::ImageTransport(nodeHandle_);

  // Setup publishers.
  setUpPublishers();

  // Setup subscribers.
  setUpSubscribers();

  MELO_INFO_STREAM("Initialized node for colorizing the point cloud.");
}

void PointCloudColorizerRos::setUpSubscribers() {
  // Input point cloud subscriber.
  pointCloudSub_ = nodeHandle_.subscribe(parameters_.inputPointCloudTopic_, 10, &PointCloudColorizerRos::inputPointCloudCallback, this);

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

  // Create camera image buffer
  cameraIdToImageBufferMap_[id].imageCache_.setCacheSize(parameters_.imgCacheSize_);
  cameraIdToImageBufferMap_[id].imageSub_ =
      imTransport_.subscribe(imageTopicName, 10, boost::bind(&PointCloudColorizerRos::inputImageCallback, this, _1, id));
  std::cout << "Initialized subscriber for camera " << id << " with topic: " << imageTopicName << std::endl;
}

void PointCloudColorizerRos::inputImageCallback(const sensor_msgs::ImageConstPtr& image, unsigned int id) {
  MELO_DEBUG_STREAM("Received image from camera " << id << " Encoding: " << image->encoding);
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
      // Height and width
      colorizer_.cameraParameters_[cameraIdToImageBufferMapIter.first].imgHeight_ = cameraInfoMsgPtr->height;
      colorizer_.cameraParameters_[cameraIdToImageBufferMapIter.first].imgWidth_ = cameraInfoMsgPtr->width;
      std::cout << "Camera id: " << cameraIdToImageBufferMapIter.first
                << ": height: " << colorizer_.cameraParameters_[cameraIdToImageBufferMapIter.first].imgHeight_
                << " width: " << colorizer_.cameraParameters_[cameraIdToImageBufferMapIter.first].imgWidth_ << std::endl;

      // Successfully read the camera matrix, increase the counter.
      readCameraMatrixCounter_++;
    }
  }
}

void PointCloudColorizerRos::inputPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& lidarCloud) {
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
    MELO_WARN_STREAM("No available cameras for colorization. Colorization won't be executed.");
    return;
  }
  /*
  // Construct a pointer
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl_cloud_ptr->reserve(lidarCloud->width * lidarCloud->height);

  pcl::fromROSMsg(*lidarCloud, *pcl_cloud_ptr);
  pcl::PointCloud<pcl::PointXYZRGBL> pointCloudData;
  pcl::copyPointCloud(*pcl_cloud_ptr, pointCloudData);

  // Main colorization and depth generation function
  pcl::PointCloud<pcl::PointXYZRGBL> colorizedPoints = colorizer_.colorizePoints(pointCloudData, availableCamerasVec_);
  */

  open3d::geometry::PointCloud pointCloudData;
  pointCloudData.points_.reserve(lidarCloud->width * lidarCloud->height);
  pointCloudData.colors_.reserve(lidarCloud->width * lidarCloud->height);
  open3d_conversions::rosToOpen3d(lidarCloud, pointCloudData);
  auto colorizedPoints = colorizer_.colorizePoints(pointCloudData, availableCamerasVec_);

  // Publish the colorized point cloud
  sensor_msgs::PointCloud2 pointsInPlaneMsg;
  open3d_conversions::open3dToRos(colorizedPoints, pointsInPlaneMsg);
  // pcl::toROSMsg(colorizedPoints, pointsInPlaneMsg);
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
        sensor_msgs::ImagePtr imageMsg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, image).toImageMsg();
        imageMsg->header = lidarCloud->header;
        imagePublishers_[id].publish(imageMsg);
      }
    }
  }

  if (colorizer_.generateDepthImages_) {
    for (size_t id = 1; id <= parameters_.numOfSupportedCameras_; id++) {
      if (depthImagePublishers_[id].getNumSubscribers() > 0u) {
        const cv::Mat depthImage{colorizer_.cameraParameters_[id].depthImgFromPoints_.clone()};
        sensor_msgs::ImagePtr depthImageMsg = cv_bridge::CvImage(std_msgs::Header(), "32FC1", depthImage).toImageMsg();

        depthImageMsg->header = imageHeaderBuffer_[id];
        depthImagePublishers_[id].publish(depthImageMsg);
      }
    }
  }
}

void PointCloudColorizerRos::readImageAndTransformation(const std::vector<unsigned int>& availableCameraInfosVec) {
  availableCamerasVec_ = availableCameraInfosVec;
  const ros::Time lidarCloudTimeStamp = currentLidarCloud_->header.stamp;
  auto vecIterator = availableCamerasVec_.begin();
  while (vecIterator != availableCamerasVec_.end()) {
    if (!synchronizeImage(*vecIterator)) {
      MELO_WARN_STREAM("Cannot obtain latest image from Camera " << *vecIterator << " for input point cloud.");
      vecIterator = availableCamerasVec_.erase(vecIterator);
      continue;
    }
    // Store the header
    imageHeaderBuffer_[*vecIterator] = tmpLatestImage_->header;
    try {
      // RGB Vs BGR doesnt make any difference in the code. But for the user nodes it does.
      MELO_DEBUG_STREAM("Encoding of compressed image is of type: " << tmpLatestImage_->encoding);
      cv_bridge::CvImagePtr cvImgPtr;
      if (tmpLatestImage_->encoding == "bayer_rggb8") {
        cvImgPtr = cv_bridge::toCvCopy(tmpLatestImage_, sensor_msgs::image_encodings::BAYER_RGGB8);
        cv::cvtColor(cvImgPtr->image, cvImgPtr->image, cv::COLOR_BayerBG2BGR);
      } else {
        cvImgPtr = cv_bridge::toCvCopy(tmpLatestImage_, sensor_msgs::image_encodings::BGR8);
      }
      MELO_DEBUG_STREAM("Received image with size: " << cvImgPtr->image.size());
      colorizer_.cameraParameters_[*vecIterator].cvImg_ = cvImgPtr->image;
      cv::imwrite("/home/peyschen/Downloads/test_color.jpg", colorizer_.cameraParameters_[*vecIterator].cvImg_);
    } catch (cv_bridge::Exception& e) {
      MELO_ERROR_STREAM("cv_bridge exception: " << e.what());
      vecIterator = availableCamerasVec_.erase(vecIterator);
      continue;
    }

    std::string targetFrame = (parameters_.useHelperFrame_) ? (imageHeaderBuffer_[*vecIterator].frame_id + "_helper")
                                                            : (imageHeaderBuffer_[*vecIterator].frame_id);

    //! Find the transformation between currentLidarCloud_ and camera.
    if (!tfBuffer_.canTransform(targetFrame, imageHeaderBuffer_[*vecIterator].stamp, currentLidarCloud_->header.frame_id,
                                lidarCloudTimeStamp, parameters_.fixedFrame_, ros::Duration(parameters_.findTfTimeOut_))) {
      MELO_WARN_STREAM("Timeout for finding the transformation between current input point cloud ["
                       << currentLidarCloud_->header.frame_id << "] and Camera " << *vecIterator << " frame ["
                       << imageHeaderBuffer_[*vecIterator].frame_id << "].");
      vecIterator = availableCamerasVec_.erase(vecIterator);
      continue;
    } else {
      // Currently on Cerberus there is a `camX_sensor_frame_helper` frame which contains the proper transformation.
      // This was different in the first mission.
      auto lidarToCameraTrans =
          tfBuffer_.lookupTransform(targetFrame, imageHeaderBuffer_[*vecIterator].stamp, currentLidarCloud_->header.frame_id,
                                    lidarCloudTimeStamp, parameters_.fixedFrame_, ros::Duration(parameters_.findTfTimeOut_));

      // Set the transformations to their respective cameras.
      colorizer_.cameraParameters_[*vecIterator].pclTransform_ = lidarToCameraTrans;
    }
    vecIterator++;
  }
}

bool PointCloudColorizerRos::synchronizeImage(int id) {
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
      MELO_WARN_STREAM("Cannot obtain latest previous image from Camera " << id << " for input point cloud. ");
      continue;
    }
    double timeDifference;
    timeDifference = (lidarCloudTimeStamp - tmpLatestImage_->header.stamp).toSec();
    MELO_DEBUG_STREAM("The timestamp difference between latest previous image from Camera "
                      << id << " and input point cloud: " << timeDifference);
    if (std::fabs(timeDifference) > parameters_.allowedCameraPointcloudTimeDiff_) {
      MELO_WARN_STREAM("The timestamp difference between latest previous image from Camera " << id << " and input point cloud is too big.");
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

  // We won't look for synchronization of this camera until a new image arrives for this camera.
  isCameraAlreadyProcessed_[id] = true;
  return true;
}
}  // namespace point_cloud_colorizer_ros
