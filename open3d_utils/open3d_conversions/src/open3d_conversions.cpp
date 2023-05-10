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
#include "open3d/core/EigenConverter.h"

namespace open3d_conversions {
void open3dToRos(const open3d::geometry::PointCloud& pointcloud, sensor_msgs::PointCloud2& ros_pc2, std::string frame_id) {
  sensor_msgs::PointCloud2Modifier modifier(ros_pc2);
  if (pointcloud.HasColors()) {
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  } else {
    modifier.setPointCloud2FieldsByString(1, "xyz");
  }
  modifier.resize(pointcloud.points_.size());
  ros_pc2.header.frame_id = frame_id;
  sensor_msgs::PointCloud2Iterator<float> ros_pc2_x(ros_pc2, "x");
  sensor_msgs::PointCloud2Iterator<float> ros_pc2_y(ros_pc2, "y");
  sensor_msgs::PointCloud2Iterator<float> ros_pc2_z(ros_pc2, "z");
  if (pointcloud.HasColors()) {
    sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_r(ros_pc2, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_g(ros_pc2, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_b(ros_pc2, "b");
    for (size_t i = 0; i < pointcloud.points_.size(); i++, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z, ++ros_pc2_r, ++ros_pc2_g, ++ros_pc2_b) {
      const Eigen::Vector3d& point = pointcloud.points_[i];
      const Eigen::Vector3d& color = pointcloud.colors_[i];
      *ros_pc2_x = point(0);
      *ros_pc2_y = point(1);
      *ros_pc2_z = point(2);
      *ros_pc2_r = (int)(255 * color(0));
      *ros_pc2_g = (int)(255 * color(1));
      *ros_pc2_b = (int)(255 * color(2));
    }
  } else {
    for (size_t i = 0; i < pointcloud.points_.size(); i++, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z) {
      const Eigen::Vector3d& point = pointcloud.points_[i];
      *ros_pc2_x = point(0);
      *ros_pc2_y = point(1);
      *ros_pc2_z = point(2);
    }
  }
}

void rosToOpen3d(const sensor_msgs::PointCloud2ConstPtr& ros_pc2, open3d::geometry::PointCloud& o3d_pc, bool skip_colors) {
  rosToOpen3d(*ros_pc2, o3d_pc, skip_colors);
}

void rosToOpen3d(const sensor_msgs::PointCloud2& cloud, open3d::geometry::PointCloud& o3d_pc, bool skip_colors) {
  const auto ros_pc2 = &cloud;
  sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_x(*ros_pc2, "x");
  sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_y(*ros_pc2, "y");
  sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_z(*ros_pc2, "z");
  o3d_pc.points_.reserve(ros_pc2->height * ros_pc2->width);
  if (ros_pc2->fields.size() == 3 || skip_colors == true) {
    for (size_t i = 0; i < ros_pc2->height * ros_pc2->width; ++i, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z) {
      o3d_pc.points_.push_back(Eigen::Vector3d(*ros_pc2_x, *ros_pc2_y, *ros_pc2_z));
    }
  } else {
    o3d_pc.colors_.reserve(ros_pc2->height * ros_pc2->width);
    if (ros_pc2->fields[3].name == "rgb") {
      sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_r(*ros_pc2, "r");
      sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_g(*ros_pc2, "g");
      sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_b(*ros_pc2, "b");

      for (size_t i = 0; i < ros_pc2->height * ros_pc2->width;
           ++i, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z, ++ros_pc2_r, ++ros_pc2_g, ++ros_pc2_b) {
        o3d_pc.points_.push_back(Eigen::Vector3d(*ros_pc2_x, *ros_pc2_y, *ros_pc2_z));
        o3d_pc.colors_.push_back(Eigen::Vector3d(((int)(*ros_pc2_r)) / 255.0, ((int)(*ros_pc2_g)) / 255.0, ((int)(*ros_pc2_b)) / 255.0));
      }
    } else if (ros_pc2->fields[3].name == "intensity") {
      sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_i(*ros_pc2, "intensity");
      for (size_t i = 0; i < ros_pc2->height * ros_pc2->width; ++i, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z, ++ros_pc2_i) {
        o3d_pc.points_.push_back(Eigen::Vector3d(*ros_pc2_x, *ros_pc2_y, *ros_pc2_z));
        o3d_pc.colors_.push_back(Eigen::Vector3d(*ros_pc2_i, *ros_pc2_i, *ros_pc2_i));
      }
    }
  }
}
void open3dToRos(const open3d::t::geometry::PointCloud& pointcloud, sensor_msgs::PointCloud2& ros_pc2, std::string frame_id,
                 int t_num_fields, ...) {
  sensor_msgs::PointCloud2Modifier modifier(ros_pc2);
  ros_pc2.fields.reserve(t_num_fields);
  va_list vl;
  va_start(vl, t_num_fields);
  int offset = 0;
  std::vector<std::string> field_names;
  std::vector<std::string> data_types;
  for (int i = 0; i < t_num_fields - 1; ++i) {
    std::string field_name = std::string(va_arg(vl, char*));
    field_names.push_back(field_name);
    i++;
    std::string data_type = std::string(va_arg(vl, char*));
    data_types.push_back(data_type);
    if (field_name == "xyz") {
      modifier.setPointCloud2FieldsByString(1, "xyz");
    } else {
      if ((field_name == "rgb") || (field_name == "rgba")) {
        modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
      } else {
        if (data_type == "float") {
          offset = addPointField(ros_pc2, field_name + "_x", 1, sensor_msgs::PointField::FLOAT32, offset);
          offset = addPointField(ros_pc2, field_name + "_y", 1, sensor_msgs::PointField::FLOAT32, offset);
          offset = addPointField(ros_pc2, field_name + "_z", 1, sensor_msgs::PointField::FLOAT32, offset);
          offset += sizeOfPointField(sensor_msgs::PointField::FLOAT32);
        } else if (data_type == "int") {
          offset = addPointField(ros_pc2, field_name + "_x", 1, sensor_msgs::PointField::INT8, offset);
          offset = addPointField(ros_pc2, field_name + "_y", 1, sensor_msgs::PointField::INT8, offset);
          offset = addPointField(ros_pc2, field_name + "_z", 1, sensor_msgs::PointField::INT8, offset);
          offset += sizeOfPointField(sensor_msgs::PointField::INT8);
        } else {
          throw std::runtime_error("datatype" + data_type + " does not exist");
        }
      }
    }
    va_end(vl);
  }
  const open3d::core::Tensor& o3d_TensorList_points = pointcloud.GetPointPositions();
  modifier.resize(pointcloud.GetPointPositions().GetShape()[0]);
  ros_pc2.data.reserve(pointcloud.GetPointPositions().GetShape()[0]);
  ros_pc2.header.frame_id = frame_id;
  sensor_msgs::PointCloud2Iterator<float> ros_pc2_x(ros_pc2, "x");
  sensor_msgs::PointCloud2Iterator<float> ros_pc2_y(ros_pc2, "y");
  sensor_msgs::PointCloud2Iterator<float> ros_pc2_z(ros_pc2, "z");
  int count = 0;
  for (auto field_name = field_names.begin(); field_name != field_names.end(); ++field_name) {
    std::string data_type = data_types[count];
    if (*field_name == "xyz") {
      for (size_t i = 0; i < o3d_TensorList_points.GetShape()[0]; i++, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z) {
        open3d::core::Tensor point = o3d_TensorList_points[i];
        *ros_pc2_x = point[0].Item<float>();
        *ros_pc2_y = point[1].Item<float>();
        *ros_pc2_z = point[2].Item<float>();
      }
    } else if (*field_name == "rgb") {
      const open3d::core::Tensor& o3d_TensorList_colors = pointcloud.GetPointAttr("colors");
      sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_r(ros_pc2, "r");
      sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_g(ros_pc2, "g");
      sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_b(ros_pc2, "b");
      for (size_t i = 0; i < o3d_TensorList_points.GetShape()[0]; i++, ++ros_pc2_r, ++ros_pc2_g, ++ros_pc2_b) {
        open3d::core::Tensor color = o3d_TensorList_colors[i];
        *ros_pc2_r = (int)(255 * color[0].Item<float>());
        *ros_pc2_g = (int)(255 * color[1].Item<float>());
        *ros_pc2_b = (int)(255 * color[2].Item<float>());
      }
    } else {
      const open3d::core::Tensor& o3d_TensorList_fields = pointcloud.GetPointAttr(*field_name);
      if (data_type == "int") {
        sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_fx(ros_pc2, *field_name + "_x");
        sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_fy(ros_pc2, *field_name + "_y");
        sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_fz(ros_pc2, *field_name + "_z");
        for (size_t i = 0; i < o3d_TensorList_points.GetShape()[0];
             i++, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z, ++ros_pc2_fx, ++ros_pc2_fy, ++ros_pc2_fz) {
          open3d::core::Tensor field_tensor = o3d_TensorList_points[i];
          *ros_pc2_fx = field_tensor[0].Item<int>();
          *ros_pc2_fy = field_tensor[1].Item<int>();
          *ros_pc2_fz = field_tensor[2].Item<int>();
        }
      } else if (data_type == "float") {
        sensor_msgs::PointCloud2Iterator<float> ros_pc2_fx(ros_pc2, *field_name + "_x");
        sensor_msgs::PointCloud2Iterator<float> ros_pc2_fy(ros_pc2, *field_name + "_y");
        sensor_msgs::PointCloud2Iterator<float> ros_pc2_fz(ros_pc2, *field_name + "_z");
        for (size_t i = 0; i < o3d_TensorList_points.GetShape()[0];
             i++, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z, ++ros_pc2_fx, ++ros_pc2_fy, ++ros_pc2_fz) {
          open3d::core::Tensor field_tensor = o3d_TensorList_points[i];
          *ros_pc2_fx = field_tensor[0].Item<float>();
          *ros_pc2_fy = field_tensor[1].Item<float>();
          *ros_pc2_fz = field_tensor[2].Item<float>();
        }
      }
    }
    count++;
  }
}

void rosToOpen3d(const sensor_msgs::PointCloud2ConstPtr& ros_pc2, open3d::t::geometry::PointCloud& o3d_tpc, bool skip_colors) {
  sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_x(*ros_pc2, "x");
  sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_y(*ros_pc2, "y");
  sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_z(*ros_pc2, "z");
  open3d::core::Dtype dtype_f = open3d::core::Dtype::Float32;
  open3d::core::Dtype dtype_lf = open3d::core::Dtype::Float64;
  open3d::core::Device device_type(open3d::core::Device::DeviceType::CPU, 0);
  std::vector<Eigen::Vector3d> o3d_TensorList_points;

  for (int num_fields = 0; num_fields < ros_pc2->fields.size(); num_fields++) {
    if (ros_pc2->fields[num_fields].name == "x") {
      for (size_t i = 0; i < ros_pc2->height * ros_pc2->width; ++i, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z) {
        o3d_TensorList_points.push_back(Eigen::Vector3d(*ros_pc2_x, *ros_pc2_y, *ros_pc2_z));
      }
      open3d::core::Tensor o3d_tpc_points =
          open3d::core::eigen_converter::EigenVector3dVectorToTensor(o3d_TensorList_points, dtype_f, device_type);
      o3d_tpc.SetPointPositions(o3d_tpc_points);
      num_fields++;
      num_fields++;
    } else if (ros_pc2->fields[num_fields].name == "rgb" && !skip_colors) {
      sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_r(*ros_pc2, "r");
      sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_g(*ros_pc2, "g");
      sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_b(*ros_pc2, "b");
      std::vector<Eigen::Vector3d> o3d_TensorList_colors;
      for (size_t i = 0; i < ros_pc2->height * ros_pc2->width; ++i, ++ros_pc2_r, ++ros_pc2_g, ++ros_pc2_b) {
        o3d_TensorList_colors.push_back(
            Eigen::Vector3d(((int)(*ros_pc2_r)) / 255.0, ((int)(*ros_pc2_g)) / 255.0, ((int)(*ros_pc2_b)) / 255.0));
      }
      open3d::core::Tensor o3d_tpc_colors =
          open3d::core::eigen_converter::EigenVector3dVectorToTensor(o3d_TensorList_colors, dtype_f, device_type);
      o3d_tpc.SetPointColors(o3d_tpc_colors);
    } else {
      if (ros_pc2->fields[num_fields].datatype == sensor_msgs::PointField::UINT8 ||
          ros_pc2->fields[num_fields].datatype == sensor_msgs::PointField::INT8) {
        sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_fx(*ros_pc2, ros_pc2->fields[num_fields].name);
        sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_fy(*ros_pc2, ros_pc2->fields[num_fields].name);
        sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_fz(*ros_pc2, ros_pc2->fields[num_fields].name);
        std::vector<Eigen::Vector3d> o3d_TensorList_fields;

        for (size_t i = 0; i < ros_pc2->height * ros_pc2->width; ++i, ++ros_pc2_fx, ++ros_pc2_fy, ++ros_pc2_fz) {
          o3d_TensorList_fields.push_back(Eigen::Vector3d(*ros_pc2_fx, *ros_pc2_fy, *ros_pc2_fz));
        }
        open3d::core::Tensor o3d_tpc_fields =
            open3d::core::eigen_converter::EigenVector3dVectorToTensor(o3d_TensorList_fields, dtype_f, device_type);
        o3d_tpc.SetPointAttr(ros_pc2->fields[num_fields].name, o3d_tpc_fields);
      } else if (ros_pc2->fields[num_fields].datatype == sensor_msgs::PointField::FLOAT32) {
        sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_fx(*ros_pc2, ros_pc2->fields[num_fields].name);
        sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_fy(*ros_pc2, ros_pc2->fields[num_fields].name);
        sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_fz(*ros_pc2, ros_pc2->fields[num_fields].name);
        std::vector<Eigen::Vector3d> o3d_TensorList_fields;

        for (size_t i = 0; i < ros_pc2->height * ros_pc2->width; ++i, ++ros_pc2_fx, ++ros_pc2_fy, ++ros_pc2_fz) {
          o3d_TensorList_fields.push_back(Eigen::Vector3d(*ros_pc2_fx, *ros_pc2_fy, *ros_pc2_fz));
        }
        open3d::core::Tensor o3d_tpc_fields =
            open3d::core::eigen_converter::EigenVector3dVectorToTensor(o3d_TensorList_fields, dtype_f, device_type);
        o3d_tpc.SetPointAttr(ros_pc2->fields[num_fields].name, o3d_tpc_fields);
      } else {
        sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_fx(*ros_pc2, ros_pc2->fields[num_fields].name);
        sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_fy(*ros_pc2, ros_pc2->fields[num_fields].name);
        sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_fz(*ros_pc2, ros_pc2->fields[num_fields].name);
        std::vector<Eigen::Vector3d> o3d_TensorList_fields;

        for (size_t i = 0; i < ros_pc2->height * ros_pc2->width; ++i, ++ros_pc2_fx, ++ros_pc2_fy, ++ros_pc2_fz) {
          o3d_TensorList_fields.push_back(Eigen::Vector3d(*ros_pc2_fx, *ros_pc2_fy, *ros_pc2_fz));
        }
        open3d::core::Tensor o3d_tpc_fields =
            open3d::core::eigen_converter::EigenVector3dVectorToTensor(o3d_TensorList_fields, dtype_f, device_type);
        o3d_tpc.SetPointAttr(ros_pc2->fields[num_fields].name, o3d_tpc_fields);
      }
    }
  }
}

void open3dToRos(const open3d::geometry::MeshBase& mesh, const std::string& frameId, open3d_slam_msgs::PolygonMesh& msg) {
  enum XYZ { x, y, z };
  // Constructing the vertices pointcloud
  using namespace open3d::geometry;
  PointCloud pointCloud;
  const int nVertices = mesh.vertices_.size();
  pointCloud.points_ = mesh.vertices_;
  open3dToRos(pointCloud, msg.cloud, frameId);

  if (mesh.GetGeometryType() != Geometry::GeometryType::TriangleMesh) {
    throw std::runtime_error("Only triangle mesh supported for now!");
  }

  const TriangleMesh& triangleMesh = static_cast<const TriangleMesh&>(mesh);

  // add triangles
  const int nTriangles = triangleMesh.triangles_.size();
  msg.polygons.reserve(nTriangles);
  for (int i = 0; i < nTriangles; ++i) {
    open3d_slam_msgs::Vertices triangle;
    triangle.vertices.resize(3);
    triangle.vertices[x] = triangleMesh.triangles_.at(i).x();
    triangle.vertices[y] = triangleMesh.triangles_.at(i).y();
    triangle.vertices[z] = triangleMesh.triangles_.at(i).z();
    msg.polygons.push_back(triangle);
  }
}

void rosToOpen3d(const open3d_slam_msgs::PolygonMesh::ConstPtr& msg, open3d::geometry::TriangleMesh& mesh) {
  rosToOpen3d(*msg, mesh);
}

void rosToOpen3d(const open3d_slam_msgs::PolygonMesh& msg, open3d::geometry::TriangleMesh& mesh) {
  using namespace open3d::geometry;
  enum XYZ { x, y, z };
  PointCloud pointCloud;

  rosToOpen3d(msg.cloud, pointCloud);
  mesh.vertices_ = pointCloud.points_;

  // add triangles
  const int nTriangles = msg.polygons.size();
  mesh.triangles_.reserve(nTriangles);
  for (int i = 0; i < nTriangles; ++i) {
    Eigen::Vector3i triangle(msg.polygons[i].vertices[x], msg.polygons[i].vertices[y], msg.polygons[i].vertices[z]);
    mesh.triangles_.push_back(triangle);
  }
}

}  // namespace open3d_conversions

inline int addPointField(sensor_msgs::PointCloud2& cloud_msg, const std::string& name, int count, int datatype, int offset)

{
  sensor_msgs::PointField point_field;
  point_field.name = name;
  point_field.count = count;
  point_field.datatype = datatype;
  point_field.offset = offset;
  cloud_msg.fields.push_back(point_field);

  // Update the offset
  return offset + point_field.count * sizeOfPointField(datatype);
}

inline int sizeOfPointField(int datatype) {
  if ((datatype == sensor_msgs::PointField::INT8) || (datatype == sensor_msgs::PointField::UINT8)) {
    return 1;
  } else if ((datatype == sensor_msgs::PointField::INT16) ||  // NOLINT
             (datatype == sensor_msgs::PointField::UINT16)) {
    return 2;
  } else if ((datatype == sensor_msgs::PointField::INT32) ||  // NOLINT
             (datatype == sensor_msgs::PointField::UINT32) || (datatype == sensor_msgs::PointField::FLOAT32)) {
    return 4;
  } else if (datatype == sensor_msgs::PointField::FLOAT64) {
    return 8;
  } else {
    std::stringstream err;
    err << "PointField of type " << datatype << " does not exist";
    throw std::runtime_error(err.str());
  }
  return -1;
}
