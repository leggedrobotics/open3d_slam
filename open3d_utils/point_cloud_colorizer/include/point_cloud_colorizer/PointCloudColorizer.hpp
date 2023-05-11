#pragma once

#include <open3d/geometry/PointCloud.h>

// Transformation
#include <geometry_msgs/PoseStamped.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

// Eigen
#include <Eigen/Core>

// OpenCV
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// message logger
#include <message_logger/message_logger.hpp>

namespace point_cloud_colorizer {

struct CameraParameters {
  //! Camera intrinsic matrix container.
  Eigen::Matrix<double, 3, 3> intrinsicCameraMatrix_;

  //! RGB image height.
  unsigned int imgHeight_{0u};

  //! RGB image width.
  unsigned int imgWidth_{0u};

  //! The latest rgb image.
  cv::Mat cvImg_;

  // The depth image from the LiDAR information
  cv::Mat depthImgFromPoints_;

  //! The latest rgb image with projected points.
  cv::Mat cameraImgWithPointCloudOverlaid_;

  // TF default message, PCL::transform can use this type
  geometry_msgs::TransformStamped pclTransform_;
};

class PointCloudColorizer {
  // PCL Types
  // using Point = pcl::PointXYZRGBL;
  // using PointCloud = pcl::PointCloud<Point>;
  // O3D
  using Point = Eigen::Vector3d;
  using PointCloud = open3d::geometry::PointCloud;
  using PointCloudPtr = std::shared_ptr<PointCloud>;
  using PointCloudConstPtr = const std::shared_ptr<PointCloud>;

 public:
  //! constructor
  PointCloudColorizer() = default;

  //! Main colorization function
  PointCloud colorizePoints(PointCloud& pointCloud, const std::vector<unsigned int>& cameraIdVec);

  // Generate depth image.
  void generateDepthImage(PointCloud& pointCloud, Eigen::MatrixXi& pointCloudToImageCoordinate, unsigned int id,
                          const std::size_t nbPoints);

  // Camera intrinsic matrices.
  std::map<unsigned int, CameraParameters> cameraParameters_;

  // Maximum depth point to colorize.
  float maxColorizationDepth_{40.0f};

  // Bloat projected pixel for more smooth changes.
  int bloatFactor_{4};

  // Whether to generate depth images.
  bool generateDepthImages_{false};

  // Whether to generate overlaid images for debugging.
  bool generateOverlaidImages_{false};

 private:
  //! Transform point cloud. Copies the data. Works per camera.
  void transformPointCloudtoCameraFrame(const PointCloud& inputpointCloud, PointCloud& outputpointCloud, unsigned int cameraId);

  //! Convert point cloud to camera frame and get the pixel coordinates on the image plane. Works per camera.
  void projectPointsToImage(const PointCloud& pointCloud, Eigen::MatrixXi& pointCloudToImageCoordinate, PointCloud& pointCloudInCameraFrame,
                            unsigned int id);

  //! Get and assign the color information from the image to the point cloud data.
  void colorizePoints(PointCloud& pointCloud, const int pointIndex, const int u, const int v, const unsigned int cameraId);

  //! Check whether the transformed point points are on the camera image plane. Expects point image pixel coordinates, the depth and the
  //! camera ID.
  bool checkDepthValidity(const double& Z, const int& u, const int& v, unsigned int id);
};
}  // namespace point_cloud_colorizer
