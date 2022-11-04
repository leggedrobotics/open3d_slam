/*
 * cloud_processing.cpp
 *
 *  Created on: Oct 21, 2022
 *      Author: jelavice
 */

#include "open3d_slam/cloud_processing.hpp"
#include "open3d_slam/croppers.hpp"


namespace o3d_slam {

PointCloudPtr cropVoxelizeDownsample(const PointCloud &in, const CroppingVolume &cropper, double voxelSize,
		double downsamplingRatio) {

	std::shared_ptr<PointCloud> cropped, voxelized, downsampled;
	cropped = cropper.crop(in);
	if (voxelSize <= 0.0) {
		voxelized = cropped;
	} else {
		voxelized = cropped->VoxelDownSample(voxelSize);
	}

	if (downsamplingRatio >= 1.0) {
		downsampled = voxelized;
	} else {
		downsampled = voxelized->RandomDownSample(downsamplingRatio);
	}

	return downsampled;

}

} // namespace o3d_slam

