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

namespace m545_mapping {

    class Color {

    public:
        Color();
        ~Color() = default;
//    void setParameters(const ColorParameters &p);

        open3d::geometry::PointCloud projectionAndColor(open3d::geometry::PointCloud &cloud,
                                                        const sensor_msgs::ImageConstPtr &msg,
                                                        const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> &K,
                                                        const Eigen::Matrix<double, 5, 1> &D,
                                                        const Eigen::Quaternion<double> &quaternion,
                                                        const Eigen::Vector3d &translation,
                                                        const bool &cropFlag);

        open3d::geometry::PointCloud getCloud2();

    private:
        std::vector<Eigen::Matrix<double, 3, 1>> imageConversion(const sensor_msgs::ImageConstPtr& msg, const std::vector<Eigen::Vector2i> pixels);

        const Eigen::Matrix<double, 3, 1> whitePoint = {1.0, 1.0, 1.0};
        std::vector<Eigen::Matrix<double, 3, 1>> pos_lidar;
        std::vector<Eigen::Matrix<double, 3, 1>> posArrayWhite;
        open3d::geometry::PointCloud cloud2;
    };

}