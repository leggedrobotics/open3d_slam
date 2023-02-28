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

#ifndef OPEN3D_CONVERSIONS_HPP_
#define OPEN3D_CONVERSIONS_HPP_

// Open3D
#include "open3d/t/geometry/PointCloud.h"
#include "open3d/geometry/PointCloud.h"
#include "open3d/geometry/TriangleMesh.h"
// ROS
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <mesh_msgs/MeshGeometry.h>
#include <geometry_msgs/Point.h>

// Eigen
#include <Eigen/Dense>

// C++
#include <string>

#include "open3d_slam_msgs/PolygonMesh.h"


namespace open3d_conversions
{
/**
 * @brief Copy data from a open3d::geometry::PointCloud to a sensor_msgs::PointCloud2
 *
 * @param pointcloud Reference to the open3d PointCloud
 * @param ros_pc2 Reference to the sensor_msgs PointCloud2
 * @param frame_id The string to be placed in the frame_id of the PointCloud2
 */
void open3dToRos(const open3d::geometry::PointCloud& pointcloud, sensor_msgs::PointCloud2& ros_pc2,
                 std::string frame_id = "open3d_pointcloud");

/**
 * @brief Copy data from a sensor_msgs::PointCloud2 to a open3d::geometry::PointCloud
 *
 * @param ros_pc2 Reference to the sensor_msgs PointCloud2
 * @param o3d_pc Reference to the open3d PointCloud
 * @param skip_colors If true, only xyz fields will be copied
 */

void rosToOpen3d(const sensor_msgs::PointCloud2ConstPtr& ros_pc2, open3d::geometry::PointCloud& o3d_pc,
                 bool skip_colors = false);
void rosToOpen3d(const sensor_msgs::PointCloud2 &cloud, open3d::geometry::PointCloud &o3d_pc,
		bool skip_colors=false);
/**
*@brief Copy data from a open3d::t::geometry::PointCloud to a sensor_msgs::PointCloud2
*
*@param pointcloud Reference to the open3d tgeometry PointCloud
*@param ros_pc2 Reference to the sensor_msgs PointCloud2
*@param frame_id The string to be placed in the frame_id of the PointCloud2
*@param t_num_fields Twice the number of fields that the pointcloud contains
*@param var_args Strings of field names followed succeeded by their datatype ("int" / "float")
*/
void open3dToRos(const open3d::t::geometry::PointCloud& pointcloud, sensor_msgs::PointCloud2& ros_pc2,
                 std::string frame_id = "open3d_pointcloud", int t_num_fields = 2, ...);

/**
 * @brief Copy data from a sensor_msgs::PointCloud2 to a open3d::t::geometry::PointCloud
 *
 * @param ros_pc2 Reference to the sensor_msgs PointCloud2
 * @param o3d_pc Reference to the open3d tgeometry PointCloud
 * @param skip_colors If true, only xyz fields will be copied
 */
void rosToOpen3d(const sensor_msgs::PointCloud2ConstPtr& ros_pc2, open3d::t::geometry::PointCloud& o3d_pc,
                 bool skip_colors = false);


void open3dToRos(const open3d::geometry::MeshBase &mesh, const std::string &frameId,  open3d_slam_msgs::PolygonMesh &msg);
void open3dToRos(const open3d::geometry::MeshBase &mesh, const std::string &frameId,  mesh_msgs::MeshGeometry &msg);



void rosToOpen3d(const open3d_slam_msgs::PolygonMesh &msg, open3d::geometry::TriangleMesh &mesh);
void rosToOpen3d(const open3d_slam_msgs::PolygonMesh::ConstPtr &msg, open3d::geometry::TriangleMesh &mesh);



}    // namespace open3d_conversions

#endif    // OPEN3D_CONVERSIONS_HPP_
