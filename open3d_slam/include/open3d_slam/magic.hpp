/*
 * magic.hpp
 *
 *  Created on: Dec 14, 2021
 *      Author: jelavice
 */

#pragma once

namespace o3d_slam {
namespace magic {
static const double voxelSizeCorrespondenceSearchIfMapVoxelSizeIsZero = 0.04;
static const size_t icpRunUntilConvergenceNumberOfIterations = 100;
static const double voxelExpansionFactorOverlapComputation = 20.0;
static const double voxelExpansionFactorIcpCorrespondenceDistance = 1.5;
static const double voxelExpansionFactorAdjacencyBasedRevisiting = 2.5;
static const size_t skipFirstNPointClouds = 5;
} // namespace magic
} // namespace o3d_slam
