//
// Created by peyschen on 08/06/23.
//
#ifndef REALTIME_MESHING_PARAMETERS_H
#define REALTIME_MESHING_PARAMETERS_H

struct MeshingParameters {
  double meshingVoxelSize_ = 0.6;
  double downsamplingVoxelSize_ = 0.3;
  double newVertexDistanceThreshold_ = 0.15;
  double sliverDeletionThreshold_ = 0.0075;
  bool shouldFilter_ = true;
  double filterEps_ = 0.01;
  double filterRadius_ = 0.3;
  int voxelMaxUpdates_ = 50;
  bool cloudInMapFrame_ = false;
  double preRotateRoll_ = 0.0;
  double preRotatePitch_ = 0.0;
  double preRotateYaw_ = 0.0;
  double meshCropHeight_ = -1.0;
};
#endif  // REALTIME_MESHING_PARAMETERS_H
