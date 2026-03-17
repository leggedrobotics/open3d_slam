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

#include "open3d_conversions/open3d_conversions.h"

#include <cstdarg>
#include <sstream>
#include <stdexcept>

#include "open3d/core/EigenConverter.h"

namespace open3d_conversions {

namespace {

int sizeOfPointField(int datatype) {
  if ((datatype == sensor_msgs::msg::PointField::INT8) || (datatype == sensor_msgs::msg::PointField::UINT8)) {
    return 1;
  }
  if ((datatype == sensor_msgs::msg::PointField::INT16) || (datatype == sensor_msgs::msg::PointField::UINT16)) {
    return 2;
  }
  if ((datatype == sensor_msgs::msg::PointField::INT32) || (datatype == sensor_msgs::msg::PointField::UINT32) ||
      (datatype == sensor_msgs::msg::PointField::FLOAT32)) {
    return 4;
  }
  if (datatype == sensor_msgs::msg::PointField::FLOAT64) {
    return 8;
  }
  std::stringstream err;
  err << "PointField of type " << datatype << " does not exist";
  throw std::runtime_error(err.str());
}

int addPointField(sensor_msgs::msg::PointCloud2& cloud_msg, const std::string& name, int count, int datatype, int offset) {
  sensor_msgs::msg::PointField point_field;
  point_field.name = name;
  point_field.count = count;
  point_field.datatype = datatype;
  point_field.offset = offset;
  cloud_msg.fields.push_back(point_field);
  return offset + point_field.count * sizeOfPointField(datatype);
}

}  // namespace

void open3dToRos(const open3d::geometry::PointCloud& pointcloud, sensor_msgs::msg::PointCloud2& ros_pc2, std::string frame_id) {
  sensor_msgs::PointCloud2Modifier modifier(ros_pc2);
  if (pointcloud.HasColors()) {
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  } else {
    modifier.setPointCloud2FieldsByString(1, "xyz");
  }
  modifier.resize(pointcloud.points_.size());
  ros_pc2.header.frame_id = std::move(frame_id);

  sensor_msgs::PointCloud2Iterator<float> ros_pc2_x(ros_pc2, "x");
  sensor_msgs::PointCloud2Iterator<float> ros_pc2_y(ros_pc2, "y");
  sensor_msgs::PointCloud2Iterator<float> ros_pc2_z(ros_pc2, "z");

  if (pointcloud.HasColors()) {
    sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_r(ros_pc2, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_g(ros_pc2, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_b(ros_pc2, "b");
    for (size_t i = 0; i < pointcloud.points_.size(); ++i, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z, ++ros_pc2_r, ++ros_pc2_g,
                ++ros_pc2_b) {
      const Eigen::Vector3d& point = pointcloud.points_[i];
      const Eigen::Vector3d& color = pointcloud.colors_[i];
      *ros_pc2_x = point(0);
      *ros_pc2_y = point(1);
      *ros_pc2_z = point(2);
      *ros_pc2_r = static_cast<int>(255 * color(0));
      *ros_pc2_g = static_cast<int>(255 * color(1));
      *ros_pc2_b = static_cast<int>(255 * color(2));
    }
    return;
  }

  for (size_t i = 0; i < pointcloud.points_.size(); ++i, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z) {
    const Eigen::Vector3d& point = pointcloud.points_[i];
    *ros_pc2_x = point(0);
    *ros_pc2_y = point(1);
    *ros_pc2_z = point(2);
  }
}

void rosToOpen3d(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& ros_pc2, open3d::geometry::PointCloud& o3d_pc, bool skip_colors) {
  rosToOpen3d(*ros_pc2, o3d_pc, skip_colors);
}

void rosToOpen3d(const sensor_msgs::msg::PointCloud2& cloud, open3d::geometry::PointCloud& o3d_pc, bool skip_colors) {
  sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_y(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_z(cloud, "z");

  o3d_pc.points_.clear();
  o3d_pc.colors_.clear();
  o3d_pc.points_.reserve(cloud.height * cloud.width);
  if (cloud.fields.size() == 3 || skip_colors) {
    for (size_t i = 0; i < cloud.height * cloud.width; ++i, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z) {
      o3d_pc.points_.emplace_back(*ros_pc2_x, *ros_pc2_y, *ros_pc2_z);
    }
    return;
  }

  o3d_pc.colors_.reserve(cloud.height * cloud.width);
  if (cloud.fields[3].name == "rgb") {
    sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_r(cloud, "r");
    sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_g(cloud, "g");
    sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_b(cloud, "b");
    for (size_t i = 0; i < cloud.height * cloud.width; ++i, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z, ++ros_pc2_r, ++ros_pc2_g,
                ++ros_pc2_b) {
      o3d_pc.points_.emplace_back(*ros_pc2_x, *ros_pc2_y, *ros_pc2_z);
      o3d_pc.colors_.emplace_back(static_cast<int>(*ros_pc2_r) / 255.0, static_cast<int>(*ros_pc2_g) / 255.0,
                                  static_cast<int>(*ros_pc2_b) / 255.0);
    }
    return;
  }

  if (cloud.fields[3].name == "intensity") {
    sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_i(cloud, "intensity");
    for (size_t i = 0; i < cloud.height * cloud.width; ++i, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z, ++ros_pc2_i) {
      o3d_pc.points_.emplace_back(*ros_pc2_x, *ros_pc2_y, *ros_pc2_z);
      o3d_pc.colors_.emplace_back(*ros_pc2_i, *ros_pc2_i, *ros_pc2_i);
    }
  }
}

void open3dToRos(const open3d::t::geometry::PointCloud& pointcloud, sensor_msgs::msg::PointCloud2& ros_pc2, std::string frame_id,
                 int t_num_fields, ...) {
  sensor_msgs::PointCloud2Modifier modifier(ros_pc2);
  ros_pc2.fields.reserve(t_num_fields);

  va_list vl;
  va_start(vl, t_num_fields);
  int offset = 0;
  std::vector<std::string> field_names;
  std::vector<std::string> data_types;
  for (int i = 0; i < t_num_fields; i += 2) {
    const std::string field_name = std::string(va_arg(vl, char*));
    const std::string data_type = std::string(va_arg(vl, char*));
    field_names.push_back(field_name);
    data_types.push_back(data_type);
    if (field_name == "xyz") {
      modifier.setPointCloud2FieldsByString(1, "xyz");
      continue;
    }
    if ((field_name == "rgb") || (field_name == "rgba")) {
      modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
      continue;
    }
    if (data_type == "float") {
      offset = addPointField(ros_pc2, field_name + "_x", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
      offset = addPointField(ros_pc2, field_name + "_y", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
      offset = addPointField(ros_pc2, field_name + "_z", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
      offset += sizeOfPointField(sensor_msgs::msg::PointField::FLOAT32);
      continue;
    }
    if (data_type == "int") {
      offset = addPointField(ros_pc2, field_name + "_x", 1, sensor_msgs::msg::PointField::INT8, offset);
      offset = addPointField(ros_pc2, field_name + "_y", 1, sensor_msgs::msg::PointField::INT8, offset);
      offset = addPointField(ros_pc2, field_name + "_z", 1, sensor_msgs::msg::PointField::INT8, offset);
      offset += sizeOfPointField(sensor_msgs::msg::PointField::INT8);
      continue;
    }
    throw std::runtime_error("datatype " + data_type + " does not exist");
  }
  va_end(vl);

  const open3d::core::Tensor& o3d_points = pointcloud.GetPointPositions();
  modifier.resize(o3d_points.GetShape()[0]);
  ros_pc2.header.frame_id = std::move(frame_id);

  sensor_msgs::PointCloud2Iterator<float> ros_pc2_x(ros_pc2, "x");
  sensor_msgs::PointCloud2Iterator<float> ros_pc2_y(ros_pc2, "y");
  sensor_msgs::PointCloud2Iterator<float> ros_pc2_z(ros_pc2, "z");

  for (size_t field_index = 0; field_index < field_names.size(); ++field_index) {
    const std::string& field_name = field_names[field_index];
    const std::string& data_type = data_types[field_index];
    if (field_name == "xyz") {
      for (int64_t i = 0; i < o3d_points.GetShape()[0]; ++i, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z) {
        const open3d::core::Tensor point = o3d_points[i];
        *ros_pc2_x = point[0].Item<float>();
        *ros_pc2_y = point[1].Item<float>();
        *ros_pc2_z = point[2].Item<float>();
      }
      continue;
    }
    if (field_name == "rgb") {
      const open3d::core::Tensor& o3d_colors = pointcloud.GetPointColors();
      sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_r(ros_pc2, "r");
      sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_g(ros_pc2, "g");
      sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_b(ros_pc2, "b");
      for (int64_t i = 0; i < o3d_points.GetShape()[0]; ++i, ++ros_pc2_r, ++ros_pc2_g, ++ros_pc2_b) {
        const open3d::core::Tensor color = o3d_colors[i];
        *ros_pc2_r = static_cast<int>(255 * color[0].Item<float>());
        *ros_pc2_g = static_cast<int>(255 * color[1].Item<float>());
        *ros_pc2_b = static_cast<int>(255 * color[2].Item<float>());
      }
      continue;
    }

    const open3d::core::Tensor& o3d_fields = pointcloud.GetPointAttr(field_name);
    if (data_type == "int") {
      sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_fx(ros_pc2, field_name + "_x");
      sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_fy(ros_pc2, field_name + "_y");
      sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_fz(ros_pc2, field_name + "_z");
      for (int64_t i = 0; i < o3d_points.GetShape()[0]; ++i, ++ros_pc2_fx, ++ros_pc2_fy, ++ros_pc2_fz) {
        const open3d::core::Tensor field_tensor = o3d_fields[i];
        *ros_pc2_fx = field_tensor[0].Item<int>();
        *ros_pc2_fy = field_tensor[1].Item<int>();
        *ros_pc2_fz = field_tensor[2].Item<int>();
      }
      continue;
    }
    if (data_type == "float") {
      sensor_msgs::PointCloud2Iterator<float> ros_pc2_fx(ros_pc2, field_name + "_x");
      sensor_msgs::PointCloud2Iterator<float> ros_pc2_fy(ros_pc2, field_name + "_y");
      sensor_msgs::PointCloud2Iterator<float> ros_pc2_fz(ros_pc2, field_name + "_z");
      for (int64_t i = 0; i < o3d_points.GetShape()[0]; ++i, ++ros_pc2_fx, ++ros_pc2_fy, ++ros_pc2_fz) {
        const open3d::core::Tensor field_tensor = o3d_fields[i];
        *ros_pc2_fx = field_tensor[0].Item<float>();
        *ros_pc2_fy = field_tensor[1].Item<float>();
        *ros_pc2_fz = field_tensor[2].Item<float>();
      }
    }
  }
}

void rosToOpen3d(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& ros_pc2, open3d::t::geometry::PointCloud& o3d_tpc,
                 bool skip_colors) {
  sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_x(*ros_pc2, "x");
  sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_y(*ros_pc2, "y");
  sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_z(*ros_pc2, "z");

  const open3d::core::Dtype dtype_f = open3d::core::Dtype::Float32;
  const open3d::core::Device device_type(open3d::core::Device::DeviceType::CPU, 0);
  std::vector<Eigen::Vector3d> o3d_points;

  for (size_t num_fields = 0; num_fields < ros_pc2->fields.size(); ++num_fields) {
    if (ros_pc2->fields[num_fields].name == "x") {
      for (size_t i = 0; i < ros_pc2->height * ros_pc2->width; ++i, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z) {
        o3d_points.emplace_back(*ros_pc2_x, *ros_pc2_y, *ros_pc2_z);
      }
      o3d_tpc.SetPointPositions(
          open3d::core::eigen_converter::EigenVector3dVectorToTensor(o3d_points, dtype_f, device_type));
      num_fields += 2;
      continue;
    }

    if (ros_pc2->fields[num_fields].name == "rgb" && !skip_colors) {
      sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_r(*ros_pc2, "r");
      sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_g(*ros_pc2, "g");
      sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_b(*ros_pc2, "b");
      std::vector<Eigen::Vector3d> o3d_colors;
      for (size_t i = 0; i < ros_pc2->height * ros_pc2->width; ++i, ++ros_pc2_r, ++ros_pc2_g, ++ros_pc2_b) {
        o3d_colors.emplace_back(static_cast<int>(*ros_pc2_r) / 255.0, static_cast<int>(*ros_pc2_g) / 255.0,
                                static_cast<int>(*ros_pc2_b) / 255.0);
      }
      o3d_tpc.SetPointColors(open3d::core::eigen_converter::EigenVector3dVectorToTensor(o3d_colors, dtype_f, device_type));
      continue;
    }

    const std::string& field_name = ros_pc2->fields[num_fields].name;
    if (ros_pc2->fields[num_fields].datatype == sensor_msgs::msg::PointField::UINT8 ||
        ros_pc2->fields[num_fields].datatype == sensor_msgs::msg::PointField::INT8) {
      sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_fx(*ros_pc2, field_name);
      sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_fy(*ros_pc2, field_name);
      sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_fz(*ros_pc2, field_name);
      std::vector<Eigen::Vector3d> o3d_fields;
      for (size_t i = 0; i < ros_pc2->height * ros_pc2->width; ++i, ++ros_pc2_fx, ++ros_pc2_fy, ++ros_pc2_fz) {
        o3d_fields.emplace_back(*ros_pc2_fx, *ros_pc2_fy, *ros_pc2_fz);
      }
      o3d_tpc.SetPointAttr(field_name, open3d::core::eigen_converter::EigenVector3dVectorToTensor(o3d_fields, dtype_f, device_type));
      continue;
    }

    sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_fx(*ros_pc2, field_name);
    sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_fy(*ros_pc2, field_name);
    sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_fz(*ros_pc2, field_name);
    std::vector<Eigen::Vector3d> o3d_fields;
    for (size_t i = 0; i < ros_pc2->height * ros_pc2->width; ++i, ++ros_pc2_fx, ++ros_pc2_fy, ++ros_pc2_fz) {
      o3d_fields.emplace_back(*ros_pc2_fx, *ros_pc2_fy, *ros_pc2_fz);
    }
    o3d_tpc.SetPointAttr(field_name, open3d::core::eigen_converter::EigenVector3dVectorToTensor(o3d_fields, dtype_f, device_type));
  }
}

void open3dToRos(const open3d::geometry::MeshBase& mesh, const std::string& frameId, open3d_slam_msgs::msg::PolygonMesh& msg) {
  using namespace open3d::geometry;
  enum XYZ { x, y, z };

  PointCloud pointCloud;
  pointCloud.points_ = mesh.vertices_;
  open3dToRos(pointCloud, msg.cloud, frameId);

  if (mesh.GetGeometryType() != Geometry::GeometryType::TriangleMesh) {
    throw std::runtime_error("Only triangle mesh supported for now");
  }

  const TriangleMesh& triangleMesh = static_cast<const TriangleMesh&>(mesh);
  msg.polygons.clear();
  msg.polygons.reserve(triangleMesh.triangles_.size());
  for (size_t i = 0; i < triangleMesh.triangles_.size(); ++i) {
    open3d_slam_msgs::msg::Vertices triangle;
    triangle.vertices.resize(3);
    triangle.vertices[x] = triangleMesh.triangles_[i].x();
    triangle.vertices[y] = triangleMesh.triangles_[i].y();
    triangle.vertices[z] = triangleMesh.triangles_[i].z();
    msg.polygons.push_back(triangle);
  }
}

void rosToOpen3d(const open3d_slam_msgs::msg::PolygonMesh::ConstSharedPtr& msg, open3d::geometry::TriangleMesh& mesh) {
  rosToOpen3d(*msg, mesh);
}

void rosToOpen3d(const open3d_slam_msgs::msg::PolygonMesh& msg, open3d::geometry::TriangleMesh& mesh) {
  using namespace open3d::geometry;
  enum XYZ { x, y, z };

  PointCloud pointCloud;
  rosToOpen3d(msg.cloud, pointCloud);
  mesh.vertices_ = pointCloud.points_;

  mesh.triangles_.clear();
  mesh.triangles_.reserve(msg.polygons.size());
  for (size_t i = 0; i < msg.polygons.size(); ++i) {
    mesh.triangles_.emplace_back(msg.polygons[i].vertices[x], msg.polygons[i].vertices[y], msg.polygons[i].vertices[z]);
  }
}

}  // namespace open3d_conversions
