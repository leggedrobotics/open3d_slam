
// Include library header.
#include "point_cloud_colorizer/PointCloudColorizer.hpp"

namespace point_cloud_colorizer {

void PointCloudColorizer::generateDepthImage(PointCloud& pointCloud, Eigen::MatrixXi& pointCloudToImageCoordinate, unsigned int id,
                                             const std::size_t nbPoints) {
  cameraParameters_[id].depthImgFromPoints_ = cv::Mat::zeros(cameraParameters_[id].imgHeight_, cameraParameters_[id].imgWidth_, CV_32FC1);
  cameraParameters_[id].depthImgFromPoints_.setTo(0.0f);

  const auto width = int(cameraParameters_[id].imgWidth_);
  const auto height = int(cameraParameters_[id].imgHeight_);

  // Iterate through all the points
  for (size_t ii = 0; ii < nbPoints; ii++) {
    cv::Mat* local_result;
    local_result = &(cameraParameters_[id].depthImgFromPoints_);

    const float Z = pointCloud.at(ii).z;

    // Fetch image coordinates of point cloud.
    const int u = int(pointCloudToImageCoordinate.col(ii)[0]);
    const int v = int(pointCloudToImageCoordinate.col(ii)[1]);

    if (!checkDepthValidity(Z, u, v, id)) {
      continue;
    }

    // Bloat points in the image using bloat_factor
    if (u - bloatFactor_ >= 0 && u + bloatFactor_ < width && v - bloatFactor_ >= 0 && v + bloatFactor_ < height) {
      const auto cur_depth = static_cast<float>(Z);
      for (int i = -bloatFactor_; i <= bloatFactor_; i++) {
        for (int j = -bloatFactor_; j <= bloatFactor_; j++) {
          // get the previous depth
          const auto prev_depth = local_result->at<float>(v + i, u + j);
          // update if no previous depth or closer than previous depth
          // Cap values to 254 so that 255 can represent 0 for easy reduction

          if ((prev_depth == 0.0f || cur_depth < prev_depth) && cur_depth < 65535) {
            local_result->at<float>(v + i, u + j) = cur_depth;
          }
        }
      }
    }

  }  // per point
}

pcl::PointCloud<pcl::PointXYZRGBL> PointCloudColorizer::colorizePoints(PointCloud& pointCloud,
                                                                       const std::vector<unsigned int>& cameraIdVec) {
  // Read point cloud size.
  const size_t pointCloudSize = pointCloud.size();

  // Get camera vector size. Initialize the containers.
  const size_t numOfCameras{cameraIdVec.size()};
  std::vector<Eigen::MatrixXi> pointCloudToImageCoordinate(numOfCameras);
  std::vector<PointCloud> pmCloudInCameraFrame(numOfCameras);
  unsigned int j = 0;

  // Projection of point cloud as image.
  for (size_t cameraIndex = 0; cameraIndex < numOfCameras; cameraIndex++) {
    projectPointsToImage(pointCloud, pointCloudToImageCoordinate[cameraIndex], pmCloudInCameraFrame[cameraIndex], cameraIdVec[cameraIndex]);

    // If generation of depth images are required do it.
    if (generateDepthImages_) {
      MELO_DEBUG("Generating Depth Images");
      generateDepthImage(pmCloudInCameraFrame[cameraIndex], pointCloudToImageCoordinate[cameraIndex], cameraIdVec[cameraIndex],
                         pointCloudSize);
    }
  }

  // Copy camera images before overlaying point clouds.

  if (generateOverlaidImages_) {
    for (size_t k = 0; k < numOfCameras; k++) {
      cameraParameters_[cameraIdVec[k]].cameraImgWithPointCloudOverlaid_ = cameraParameters_[cameraIdVec[k]].cvImg_.clone();
    }
  }

  // Iterate over every point in the point cloud and find out if they have a matching pixel in the camera image.
  for (size_t col = 0; col < pointCloudSize; col++) {
    for (size_t k = 0; k < numOfCameras; k++) {
      // Fetch z of point from point cloud.
      const float Z = pmCloudInCameraFrame[k].at(col).z;

      // Fetch image coordinates of point cloud.
      const int u = pointCloudToImageCoordinate[k].col(col)[0];
      const int v = pointCloudToImageCoordinate[k].col(col)[1];

      // If a map point can be projected into the camera, then assign the corresponding color to this point.
      if (checkDepthValidity(Z, u, v, cameraIdVec[k])) {
        colorizePoints(pointCloud, col, u, v, j, cameraIdVec[k]);

        if (generateOverlaidImages_) {
          int red = std::min(255, (int)(255 * abs((Z - maxColorizationDepth_) / maxColorizationDepth_)));
          int green = std::min(255, (int)(255 * (1 - abs((Z - maxColorizationDepth_) / maxColorizationDepth_))));
          cv::circle(cameraParameters_[cameraIdVec[k]].cameraImgWithPointCloudOverlaid_, cv::Point(u, v), bloatFactor_,
                     cv::Scalar(0, green, red), -1);
        }

        ++j;
        break;
      }
    }
  }

  // Put the colorized points into a container, get rid of the rest.
  PointCloud pointCloudColored;

  for (size_t i = 0; i < j; i++) {
    const Point point = pointCloud.at(i);
    pointCloudColored.insert(pointCloudColored.end(), point);
  }

  return pointCloudColored;
}

void PointCloudColorizer::transformPointCloudtoCameraFrame(const PointCloud& inputPointCloud, PointCloud& outputPointCloud,
                                                           unsigned int id) {
  outputPointCloud = inputPointCloud;

  // PCL transformation functionality.
  pcl::transformPointCloud(inputPointCloud, outputPointCloud, tf2::transformToEigen(cameraParameters_[id].pclTransform_).matrix());
}

void PointCloudColorizer::projectPointsToImage(const PointCloud& pointCloud, Eigen::MatrixXi& pointCloudToImageCoordinate,
                                               PointCloud& pointCloudInCameraFrame, unsigned int id) {
  // Transform point cloud to camera frame.
  transformPointCloudtoCameraFrame(pointCloud, pointCloudInCameraFrame, id);

  // Get projection relavent information.
  const auto fx = static_cast<float>(cameraParameters_[id].intrinsicCameraMatrix_.row(0).col(0).value());
  const auto fy = static_cast<float>(cameraParameters_[id].intrinsicCameraMatrix_.row(1).col(1).value());
  const auto cx = static_cast<float>(cameraParameters_[id].intrinsicCameraMatrix_.row(0).col(2).value());
  const auto cy = static_cast<float>(cameraParameters_[id].intrinsicCameraMatrix_.row(1).col(2).value());

  const std::size_t total_pts = pointCloudInCameraFrame.size();

  // Iterate through all the points
  pointCloudToImageCoordinate = Eigen::MatrixXi::Zero(2, total_pts);
  for (std::size_t ii = 0; ii < total_pts; ii++) {
    const auto& point = pointCloudInCameraFrame.at(ii);

    // compute projected image coordinates
    pointCloudToImageCoordinate.row(0).col(ii) << static_cast<int>((fx * point.x) / point.z + cx + 0.5f);
    pointCloudToImageCoordinate.row(1).col(ii) << static_cast<int>((fy * point.y) / point.z + cy + 0.5f);
  }
}

void PointCloudColorizer::colorizePoints(PointCloud& pointCloud, int col, int u, int v, unsigned int& j, unsigned int id) {
  // Set the color value. Current order is for BGR.

  cv::Vec4b& pixel = cameraParameters_[id].cvImg_.at<cv::Vec4b>(v, u);

  pointCloud.at(col).b = static_cast<int>(pixel[0]);  // PCL does not expect a normalized range. 0-255 is fine;
  pointCloud.at(col).g = static_cast<int>(pixel[1]);
  pointCloud.at(col).r = static_cast<int>(pixel[2]);
  pointCloud.at(col).label = static_cast<int>(pixel[3]);

  /*
  if (static_cast<int>(pixel[3]) == 1 || (static_cast<int>(pixel[3]) == 2)) {
    pointCloud.at(col).b = 0;
    pointCloud.at(col).g = 255;
    pointCloud.at(col).r = 0;
    // pointCloud.at(col).a = 255;
  }

  if ((static_cast<int>(pixel[3]) == 0)) {
    pointCloud.at(col).b = 255;
    pointCloud.at(col).g = 0;
    pointCloud.at(col).r = 0;
    // pointCloud.at(col).a = 255;
  }

  if ((static_cast<int>(pixel[3]) == 255)) {
    pointCloud.at(col).b = 0;
    pointCloud.at(col).g = 0;
    pointCloud.at(col).r = 255;
    // pointCloud.at(col).a = 255;
  }

  if ((static_cast<int>(pixel[3]) == 3)) {
    pointCloud.at(col).b = 0;
    pointCloud.at(col).g = 255;
    pointCloud.at(col).r = 255;
    // pointCloud.at(col).a = 255;
  }
  */

  // Re-assign point cloud features and descriptors based on indices.
  pointCloud.at(j) = pointCloud.at(col);
}

bool PointCloudColorizer::checkDepthValidity(const float& Z, const int& u, const int& v, unsigned int id) {
  bool isValid{true};

  isValid &= Z > 0;                      // if behind camera
  isValid &= Z < maxColorizationDepth_;  // if too far away.
  isValid &= u >= 0;                     // if out of the scope of image
  isValid &= v >= 0;                     // if out of the scope of image.
  isValid &= u < int(cameraParameters_[id].imgWidth_);
  isValid &= v < int(cameraParameters_[id].imgHeight_);

  return isValid;
}

}  // namespace point_cloud_colorizer
