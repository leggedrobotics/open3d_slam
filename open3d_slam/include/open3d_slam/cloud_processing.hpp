/*
 * CloudProcessing.hpp
 *
 *  Created on: Oct 21, 2022
 *      Author: jelavice
 */

#pragma once
#include <open3d/geometry/PointCloud.h>
#include "open3d_slam/typedefs.hpp"
#include <memory>

namespace o3d_slam {

class CroppingVolume;

PointCloudPtr cropVoxelizeDownsample(const PointCloud &in,const CroppingVolume &cropper, double voxelSize,
		double downsamplingRatio);

} // namespace o3d_slam
