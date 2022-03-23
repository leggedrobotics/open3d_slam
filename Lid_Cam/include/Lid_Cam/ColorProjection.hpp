#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <open3d/geometry/PointCloud.h>
#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <cv_bridge/rgb_colors.h>
#include "opencv2/core/eigen.hpp"

namespace o3d_slam {

    class ColorProjection {

    public:
        ColorProjection();
        ~ColorProjection() = default;
//    void setParameters(const ColorParameters &p);

        open3d::geometry::PointCloud projectionAndColor(open3d::geometry::PointCloud &cloud,
                                                        const sensor_msgs::ImageConstPtr &msg,
                                                        const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> &K,
                                                        const Eigen::Matrix<double, 5, 1> &D,
                                                        const Eigen::Vector3d &rpy,
                                                        const Eigen::Vector3d &translation,
                                                        const bool &cropFlag);

        open3d::geometry::PointCloud filterColor(const open3d::geometry::PointCloud& cloud);
        open3d::geometry::RGBDImage getRGBDImage(const std::vector<double>& depth, const sensor_msgs::ImageConstPtr &msg);
        std::vector<double> getDepthInfo();



    private:
        std::vector<Eigen::Matrix<double, 3, 1>> imageConversion(const sensor_msgs::ImageConstPtr& msg, const std::vector<Eigen::Vector2i> pixels);
        const Eigen::Matrix<double, 3, 1> noColor_ = {-1.0, -1.0, -1.0};
        std::vector<Eigen::Matrix<double, 3, 1>> pos_lidar;
        std::vector<Eigen::Vector3d> getDepth(const std::vector<Eigen::Vector2i>& pixels, const std::vector<double>& depth);
        std::vector<Eigen::Vector3d> depthInfo;
    };

}
