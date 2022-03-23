// Copyright 2020 Autonomous Robots Lab, University of Nevada, Reno

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// GTest
#include <gtest/gtest.h>

// open3d_conversions
#include "open3d_conversions/open3d_conversions.h"

// Open3D
#include <open3d/Open3D.h>

// ROS
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

// Boost
#include <boost/make_shared.hpp>

TEST(ConversionFunctions, open3dToRos_uncolored)
{
  open3d::geometry::PointCloud o3d_pc;
  for (int i = 0; i < 5; ++i)
  {
    o3d_pc.points_.push_back(Eigen::Vector3d(0.5 * i, i * i, 10.5 * i));
  }
  sensor_msgs::PointCloud2 ros_pc2;
  open3d_conversions::open3dToRos(o3d_pc, ros_pc2, "o3d_frame");
  EXPECT_EQ(ros_pc2.height * ros_pc2.width, o3d_pc.points_.size());
  sensor_msgs::PointCloud2Iterator<float> ros_pc2_x(ros_pc2, "x");
  sensor_msgs::PointCloud2Iterator<float> ros_pc2_y(ros_pc2, "y");
  sensor_msgs::PointCloud2Iterator<float> ros_pc2_z(ros_pc2, "z");
  for (int i = 0; i < 5; i++, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z)
  {
    const Eigen::Vector3d& point = o3d_pc.points_[i];
    EXPECT_EQ(*ros_pc2_x, 0.5 * i);
    EXPECT_EQ(*ros_pc2_y, i * i);
    EXPECT_EQ(*ros_pc2_z, 10.5 * i);
  }
}

TEST(ConversionFunctions, open3dToRos_colored)
{
  open3d::geometry::PointCloud o3d_pc;
  for (int i = 0; i < 5; ++i)
  {
    o3d_pc.points_.push_back(Eigen::Vector3d(0.5 * i, i * i, 10.5 * i));
    o3d_pc.colors_.push_back(Eigen::Vector3d(2 * i / 255.0, 5 * i / 255.0, 10 * i / 255.0));
  }
  sensor_msgs::PointCloud2 ros_pc2;
  open3d_conversions::open3dToRos(o3d_pc, ros_pc2, "o3d_frame");
  EXPECT_EQ(ros_pc2.height * ros_pc2.width, o3d_pc.points_.size());
  sensor_msgs::PointCloud2Iterator<float> ros_pc2_x(ros_pc2, "x");
  sensor_msgs::PointCloud2Iterator<float> ros_pc2_y(ros_pc2, "y");
  sensor_msgs::PointCloud2Iterator<float> ros_pc2_z(ros_pc2, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_r(ros_pc2, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_g(ros_pc2, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_b(ros_pc2, "b");
  for (int i = 0; i < 5; i++, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z, ++ros_pc2_r, ++ros_pc2_g, ++ros_pc2_b)
  {
    const Eigen::Vector3d& point = o3d_pc.points_[i];
    EXPECT_EQ(*ros_pc2_x, 0.5 * i);
    EXPECT_EQ(*ros_pc2_y, i * i);
    EXPECT_EQ(*ros_pc2_z, 10.5 * i);
    const Eigen::Vector3d& color = o3d_pc.points_[i];
    EXPECT_EQ(*ros_pc2_r, 2 * i);
    EXPECT_EQ(*ros_pc2_g, 5 * i);
    EXPECT_EQ(*ros_pc2_b, 10 * i);
  }
}

TEST(ConversionFunctions, rosToOpen3d_uncolored)
{
  sensor_msgs::PointCloud2 ros_pc2;
  ros_pc2.header.frame_id = "ros";
  ros_pc2.height = 1;
  ros_pc2.width = 5;
  ros_pc2.is_bigendian = false;
  ros_pc2.is_dense = true;
  sensor_msgs::PointCloud2Modifier modifier(ros_pc2);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(5 * 1);
  sensor_msgs::PointCloud2Iterator<float> mod_x(ros_pc2, "x");
  sensor_msgs::PointCloud2Iterator<float> mod_y(ros_pc2, "y");
  sensor_msgs::PointCloud2Iterator<float> mod_z(ros_pc2, "z");

  for (int i = 0; i < 5; ++i, ++mod_x, ++mod_y, ++mod_z)
  {
    *mod_x = 0.5 * i;
    *mod_y = i * i;
    *mod_z = 10.5 * i;
  }

  const sensor_msgs::PointCloud2ConstPtr& ros_pc2_ptr = boost::make_shared<sensor_msgs::PointCloud2>(ros_pc2);
  open3d::geometry::PointCloud o3d_pc;
  open3d_conversions::rosToOpen3d(ros_pc2_ptr, o3d_pc);
  EXPECT_EQ(ros_pc2_ptr->height * ros_pc2_ptr->width, o3d_pc.points_.size());
  EXPECT_EQ(o3d_pc.HasColors(), false);
  for (unsigned int i = 0; i < 5; i++)
  {
    const Eigen::Vector3d& point = o3d_pc.points_[i];
    EXPECT_EQ(point(0), 0.5 * i);
    EXPECT_EQ(point(1), i * i);
    EXPECT_EQ(point(2), 10.5 * i);
  }
}

TEST(ConversionFunctions, rosToOpen3d_colored)
{
  sensor_msgs::PointCloud2 ros_pc2;
  ros_pc2.header.frame_id = "ros";
  ros_pc2.height = 1;
  ros_pc2.width = 5;
  ros_pc2.is_bigendian = false;
  ros_pc2.is_dense = true;
  sensor_msgs::PointCloud2Modifier modifier(ros_pc2);
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  modifier.resize(5 * 1);

  sensor_msgs::PointCloud2Iterator<float> mod_x(ros_pc2, "x");
  sensor_msgs::PointCloud2Iterator<float> mod_y(ros_pc2, "y");
  sensor_msgs::PointCloud2Iterator<float> mod_z(ros_pc2, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> mod_r(ros_pc2, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> mod_g(ros_pc2, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> mod_b(ros_pc2, "b");

  for (int i = 0; i < 5; ++i, ++mod_x, ++mod_y, ++mod_z, ++mod_r, ++mod_g, ++mod_b)
  {
    *mod_x = 0.5 * i;
    *mod_y = i * i;
    *mod_z = 10.5 * i;
    *mod_r = 2 * i;
    *mod_g = 5 * i;
    *mod_b = 10 * i;
  }

  const sensor_msgs::PointCloud2ConstPtr& ros_pc2_ptr = boost::make_shared<sensor_msgs::PointCloud2>(ros_pc2);
  open3d::geometry::PointCloud o3d_pc;
  open3d_conversions::rosToOpen3d(ros_pc2_ptr, o3d_pc);
  EXPECT_EQ(ros_pc2_ptr->height * ros_pc2_ptr->width, o3d_pc.points_.size());
  EXPECT_EQ(o3d_pc.HasColors(), true);
  for (unsigned int i = 0; i < 5; i++)
  {
    const Eigen::Vector3d& point = o3d_pc.points_[i];
    EXPECT_EQ(point(0), 0.5 * i);
    EXPECT_EQ(point(1), i * i);
    EXPECT_EQ(point(2), 10.5 * i);
    const Eigen::Vector3d& color = o3d_pc.colors_[i];
    EXPECT_EQ(color(0), 2 * i / 255.0);
    EXPECT_EQ(color(1), 5 * i / 255.0);
    EXPECT_EQ(color(2), 10 * i / 255.0);
  }
}

TEST(ConversionFunctionsTgeometry, open3dToRos_uncolored_tgeometry)
{
  open3d::t::geometry::PointCloud o3d_tpc;
  open3d::core::Dtype dtype_f = open3d::core::Dtype::Float32;
  open3d::core::Device device_type(open3d::core::Device::DeviceType::CPU, 0);
  std::vector<Eigen::Vector3d> o3d_vector_points;
  for (int i = 0; i < 5; ++i)
  {
    o3d_vector_points.push_back(Eigen::Vector3d(0.5 * i, i * i, 10.5 * i));
  }
  open3d::core::Tensor o3d_tpc_points =
    open3d::core::eigen_converter::EigenVector3dVectorToTensor(o3d_vector_points, dtype_f, device_type);
  o3d_tpc.SetPoints(o3d_tpc_points);
  sensor_msgs::PointCloud2 ros_pc2;
  open3d_conversions::open3dToRos(o3d_tpc, ros_pc2, "o3d_frame", 2, "xyz", "float");
  EXPECT_EQ(ros_pc2.height * ros_pc2.width, o3d_vector_points.size());
  sensor_msgs::PointCloud2Iterator<float> ros_pc2_x(ros_pc2, "x");
  sensor_msgs::PointCloud2Iterator<float> ros_pc2_y(ros_pc2, "y");
  sensor_msgs::PointCloud2Iterator<float> ros_pc2_z(ros_pc2, "z");
  for (int i = 0; i < 5; i++, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z)
  {
    EXPECT_EQ(*ros_pc2_x, 0.5 * i);
    EXPECT_EQ(*ros_pc2_y, i * i);
    EXPECT_EQ(*ros_pc2_z, 10.5 * i);
  }
}

TEST(ConversionFunctionsTgeometry, open3dToRos_colored_tgeometry)
{
  open3d::t::geometry::PointCloud o3d_tpc;
  open3d::core::Dtype dtype_f = open3d::core::Dtype::Float32;
  open3d::core::Device device_type(open3d::core::Device::DeviceType::CPU, 0);
  std::vector<Eigen::Vector3d> o3d_vector_points;
  std::vector<Eigen::Vector3d> o3d_vector_colors;

  for (int i = 0; i < 5; ++i)
  {
    o3d_vector_points.push_back(Eigen::Vector3d(0.5 * i, i * i, 10.5 * i));
    o3d_vector_colors.push_back(Eigen::Vector3d(2 * i / 255.0, 5 * i / 255.0, 10 * i / 255.0));
  }
  open3d::core::Tensor o3d_tpc_points =
    open3d::core::eigen_converter::EigenVector3dVectorToTensor(o3d_vector_points, dtype_f, device_type);
  o3d_tpc.SetPoints(o3d_tpc_points);
  open3d::core::Tensor o3d_tpc_colors =
    open3d::core::eigen_converter::EigenVector3dVectorToTensor(o3d_vector_colors, dtype_f, device_type);
  o3d_tpc.SetPointColors(o3d_tpc_colors);
  sensor_msgs::PointCloud2 ros_pc2;
  open3d_conversions::open3dToRos(o3d_tpc, ros_pc2, "o3d_frame", 4, "xyz", "float", "rgb", "float");
  EXPECT_EQ(ros_pc2.height * ros_pc2.width, o3d_tpc.GetPoints().GetShape()[0]);
  sensor_msgs::PointCloud2Iterator<float> ros_pc2_x(ros_pc2, "x");
  sensor_msgs::PointCloud2Iterator<float> ros_pc2_y(ros_pc2, "y");
  sensor_msgs::PointCloud2Iterator<float> ros_pc2_z(ros_pc2, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_r(ros_pc2, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_g(ros_pc2, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_b(ros_pc2, "b");
  for (int i = 0; i < 5; i++, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z, ++ros_pc2_r, ++ros_pc2_g, ++ros_pc2_b)
  {
    EXPECT_EQ(*ros_pc2_x, 0.5 * i);
    EXPECT_EQ(*ros_pc2_y, i * i);
    EXPECT_EQ(*ros_pc2_z, 10.5 * i);

    EXPECT_EQ(*ros_pc2_r, 2 * i);
    EXPECT_EQ(*ros_pc2_g, 5 * i);
    EXPECT_EQ(*ros_pc2_b, 10 * i);
  }
}

TEST(ConversionFunctionsTgeometry, rosToOpen3d_uncolored_tgeometry)
{
  sensor_msgs::PointCloud2 ros_pc2;
  ros_pc2.header.frame_id = "ros";
  ros_pc2.height = 1;
  ros_pc2.width = 5;
  ros_pc2.is_bigendian = false;
  ros_pc2.is_dense = true;
  sensor_msgs::PointCloud2Modifier modifier(ros_pc2);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(5 * 1);
  sensor_msgs::PointCloud2Iterator<float> mod_x(ros_pc2, "x");
  sensor_msgs::PointCloud2Iterator<float> mod_y(ros_pc2, "y");
  sensor_msgs::PointCloud2Iterator<float> mod_z(ros_pc2, "z");

  for (int i = 0; i < 5; ++i, ++mod_x, ++mod_y, ++mod_z)
  {
    *mod_x = 0.5 * i;
    *mod_y = i * i;
    *mod_z = 10.5 * i;
  }

  const sensor_msgs::PointCloud2ConstPtr& ros_pc2_ptr = boost::make_shared<sensor_msgs::PointCloud2>(ros_pc2);
  open3d::t::geometry::PointCloud o3d_tpc;
  open3d_conversions::rosToOpen3d(ros_pc2_ptr, o3d_tpc);
  open3d::core::Tensor o3d_Tensor = o3d_tpc.GetPoints();
  EXPECT_EQ(ros_pc2_ptr->height * ros_pc2_ptr->width, o3d_tpc.GetPoints().GetShape()[0]);
  EXPECT_EQ(o3d_tpc.HasPointColors(), false);
  std::vector<Eigen::Vector3d> point = open3d::core::eigen_converter::TensorToEigenVector3dVector(o3d_Tensor);
  for (unsigned int i = 0; i < 5; i++)
  {
    EXPECT_EQ(point[i](0), 0.5 * i);
    EXPECT_EQ(point[i](1), i * i);
    EXPECT_EQ(point[i](2), 10.5 * i);
  }
}

TEST(ConversionFunctionsTgeometry, rosToOpen3d_colored_tgeometry)
{
  sensor_msgs::PointCloud2 ros_pc2;
  ros_pc2.header.frame_id = "ros";
  ros_pc2.height = 1;
  ros_pc2.width = 5;
  ros_pc2.is_bigendian = false;
  ros_pc2.is_dense = true;
  sensor_msgs::PointCloud2Modifier modifier(ros_pc2);
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  modifier.resize(5 * 1);

  sensor_msgs::PointCloud2Iterator<float> mod_x(ros_pc2, "x");
  sensor_msgs::PointCloud2Iterator<float> mod_y(ros_pc2, "y");
  sensor_msgs::PointCloud2Iterator<float> mod_z(ros_pc2, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> mod_r(ros_pc2, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> mod_g(ros_pc2, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> mod_b(ros_pc2, "b");

  for (int i = 0; i < 5; ++i, ++mod_x, ++mod_y, ++mod_z, ++mod_r, ++mod_g, ++mod_b)
  {
    *mod_x = 0.5 * i;
    *mod_y = i * i;
    *mod_z = 10.5 * i;
    *mod_r = 2 * i;
    *mod_g = 5 * i;
    *mod_b = 10 * i;
  }

  const sensor_msgs::PointCloud2ConstPtr& ros_pc2_ptr = boost::make_shared<sensor_msgs::PointCloud2>(ros_pc2);
  open3d::t::geometry::PointCloud o3d_tpc;
  open3d_conversions::rosToOpen3d(ros_pc2_ptr, o3d_tpc);
  open3d::core::Tensor o3d_points = o3d_tpc.GetPoints();
  open3d::core::Tensor o3d_colors = o3d_tpc.GetPointColors();
  EXPECT_EQ(ros_pc2_ptr->height * ros_pc2_ptr->width, o3d_points.GetShape()[0]);
  EXPECT_EQ(o3d_tpc.HasPointColors(), true);
  std::vector<Eigen::Vector3d> point = open3d::core::eigen_converter::TensorToEigenVector3dVector(o3d_points);
  std::vector<Eigen::Vector3d> color = open3d::core::eigen_converter::TensorToEigenVector3dVector(o3d_colors);
  for (unsigned int i = 0; i < 5; i++)
  {
    EXPECT_EQ(point[i](0), 0.5 * i);
    EXPECT_EQ(point[i](1), i * i);
    EXPECT_EQ(point[i](2), 10.5 * i);
    EXPECT_EQ(color[i](0), (float)(2 * i / 255.0));
    EXPECT_EQ(color[i](1), (float)(5 * i / 255.0));
    EXPECT_EQ(color[i](2), (float)(10 * i / 255.0));
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}