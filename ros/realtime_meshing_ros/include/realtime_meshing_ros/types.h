//
// Created by peyschen on 26/06/23.
//

#ifndef REALTIME_MESHING_ROS_TYPES_H
#define REALTIME_MESHING_ROS_TYPES_H

#include "realtime_meshing/types.h"

struct StampedTransform{
  o3d_slam::Time time_;
  o3d_slam::Transform transform_;
};

struct StampedCloud{
  o3d_slam::Time time_;
  o3d_slam::PointCloud cloud_;
};
#endif  // REALTIME_MESHING_ROS_TYPES_H
