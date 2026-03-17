/*
 * parameter_loaders.cpp
 *
 *  Created on: Nov 10, 2022
 *      Author: jelavice
 */

#include "open3d_slam_yaml_io/parameter_loaders.hpp"

#include <stdexcept>

#include "open3d_slam/math.hpp"

namespace o3d_slam {
namespace io_yaml {

namespace {

YAML::Node requireNode(const YAML::Node& node, const std::string& key) {
  const YAML::Node child = node[key];
  if (!child.IsDefined()) {
    throw std::runtime_error("Missing required YAML key: " + key);
  }
  return child;
}

template <typename T>
T requireScalar(const YAML::Node& node, const std::string& key) {
  return requireNode(node, key).as<T>();
}

const double kDegToRad = M_PI / 180.0;

}  // namespace

// The YAML schema is intentionally concrete: every shipped sensor file is
// standalone and contains all required values. This replaces the old Lua
// include/dictionary layer with explicit, fail-fast configuration files.
void loadParameters(const YAML::Node& node, ConstantVelocityMotionCompensationParameters* p) {
  p->isUndistortInputCloud_ = requireScalar<bool>(node, "is_undistort_scan");
  p->isSpinningClockwise_ = requireScalar<bool>(node, "is_spinning_clockwise");
  p->scanDuration_ = requireScalar<double>(node, "scan_duration");
  p->numPosesVelocityEstimation_ = requireScalar<int>(node, "num_poses_vel_estimation");
}

void loadParameters(const YAML::Node& node, SavingParameters* p) {
  p->isSaveAtMissionEnd_ = requireScalar<bool>(node, "save_at_mission_end");
  p->isSaveMap_ = requireScalar<bool>(node, "save_map");
  p->isSaveSubmaps_ = requireScalar<bool>(node, "save_submaps");
  p->isSaveDenseSubmaps_ = requireScalar<bool>(node, "save_dense_submaps");
}

void loadParameters(const YAML::Node& node, PlaceRecognitionConsistencyCheckParameters* p) {
  p->maxDriftPitch_ = requireScalar<double>(node, "max_drift_pitch") * params_internal::kDegToRad;
  p->maxDriftRoll_ = requireScalar<double>(node, "max_drift_roll") * params_internal::kDegToRad;
  p->maxDriftYaw_ = requireScalar<double>(node, "max_drift_yaw") * params_internal::kDegToRad;
  p->maxDriftX_ = requireScalar<double>(node, "max_drift_x");
  p->maxDriftY_ = requireScalar<double>(node, "max_drift_y");
  p->maxDriftZ_ = requireScalar<double>(node, "max_drift_z");
}

void loadParameters(const YAML::Node& node, PlaceRecognitionParameters* p) {
  p->normalEstimationRadius_ = requireScalar<double>(node, "feature_map_normal_estimation_radius");
  p->featureVoxelSize_ = requireScalar<double>(node, "feature_voxel_size");
  p->featureRadius_ = requireScalar<double>(node, "feature_radius");
  p->featureKnn_ = requireScalar<int>(node, "feature_knn");
  p->normalKnn_ = requireScalar<int>(node, "feature_normal_knn");
  p->ransacNumIter_ = requireScalar<int>(node, "ransac_num_iter");
  p->ransacProbability_ = requireScalar<double>(node, "ransac_probability");
  p->ransacModelSize_ = requireScalar<int>(node, "ransac_model_size");
  p->ransacMaxCorrespondenceDistance_ = requireScalar<double>(node, "ransac_max_correspondence_dist");
  p->correspondenceCheckerDistance_ = requireScalar<double>(node, "ransac_correspondence_checker_distance");
  p->correspondenceCheckerEdgeLength_ = requireScalar<double>(node, "ransac_correspondence_checker_edge_length");
  p->ransacMinCorrespondenceSetSize_ = requireScalar<int>(node, "ransac_min_correspondence_set_size");
  p->maxIcpCorrespondenceDistance_ = requireScalar<double>(node, "max_icp_correspondence_distance");
  p->minRefinementFitness_ = requireScalar<double>(node, "min_icp_refinement_fitness");
  p->isDumpPlaceRecognitionAlignmentsToFile_ = requireScalar<bool>(node, "dump_aligned_place_recognitions_to_file");
  p->minSubmapsBetweenLoopClosures_ = requireScalar<int>(node, "min_submaps_between_loop_closures");
  p->loopClosureSearchRadius_ = requireScalar<double>(node, "loop_closure_search_radius");

  loadParameters(requireNode(node, "consistency_check"), &(p->consistencyCheck_));
}

void loadParameters(const YAML::Node& node, GlobalOptimizationParameters* p) {
  p->edgePruneThreshold_ = requireScalar<double>(node, "edge_prune_threshold");
  p->loopClosurePreference_ = requireScalar<double>(node, "loop_closure_preference");
  p->maxCorrespondenceDistance_ = requireScalar<double>(node, "max_correspondence_distance");
  p->referenceNode_ = requireScalar<int>(node, "reference_node");
}

void loadParameters(const YAML::Node& node, VisualizationParameters* p) {
  p->assembledMapVoxelSize_ = requireScalar<double>(node, "assembled_map_voxel_size");
  p->submapVoxelSize_ = requireScalar<double>(node, "submaps_voxel_size");
  p->visualizeEveryNmsec_ = requireScalar<double>(node, "visualize_every_n_msec");
}

void loadParameters(const YAML::Node& node, IcpParameters* p) {
  p->knn_ = requireScalar<int>(node, "knn");
  p->maxCorrespondenceDistance_ = requireScalar<double>(node, "max_correspondence_dist");
  p->maxNumIter_ = requireScalar<int>(node, "max_n_iter");
  p->maxDistanceKnn_ = requireScalar<double>(node, "max_distance_knn");
}

void loadParameters(const YAML::Node& node, CloudRegistrationParameters* p) {
  const std::string regTypeName = requireScalar<std::string>(node, "cloud_registration_type");
  p->regType_ = CloudRegistrationStringToEnumMap.at(regTypeName);
  loadParameters(requireNode(node, "icp"), &p->icp_);
}

void loadParameters(const YAML::Node& node, OdometryParameters* p) {
  loadParameters(requireNode(node, "scan_matching"), &(p->scanMatcher_));
  loadParameters(requireNode(node, "scan_processing"), &(p->scanProcessing_));
  p->isPublishOdometryMsgs_ = requireScalar<bool>(node, "is_publish_odometry_msgs");
  p->odometryBufferSize_ = requireScalar<int>(node, "odometry_buffer_size");
}

void loadParameters(const YAML::Node& node, ScanProcessingParameters* p) {
  p->voxelSize_ = requireScalar<double>(node, "voxel_size");
  p->downSamplingRatio_ = requireScalar<double>(node, "downsampling_ratio");
  p->pointCloudBufferSize_ = requireScalar<int>(node, "point_cloud_buffer_size");
  loadParameters(requireNode(node, "scan_cropping"), &(p->cropper_));
}

void loadParameters(const YAML::Node& node, ScanCroppingParameters* p) {
  p->croppingMaxRadius_ = requireScalar<double>(node, "cropping_radius_max");
  p->croppingMinRadius_ = requireScalar<double>(node, "cropping_radius_min");
  p->croppingMinZ_ = requireScalar<double>(node, "min_z");
  p->croppingMaxZ_ = requireScalar<double>(node, "max_z");
  p->cropperName_ = requireScalar<std::string>(node, "cropper_type");
}

void loadParameters(const YAML::Node& node, SubmapParameters* p) {
  p->radius_ = requireScalar<double>(node, "size");
  p->minNumRangeData_ = requireScalar<int>(node, "min_num_range_data");
  p->adjacencyBasedRevisitingMinFitness_ = requireScalar<double>(node, "adjacency_based_revisiting_min_fitness");
  p->numScansOverlap_ = requireScalar<int>(node, "submaps_num_scan_overlap");
}

void loadParameters(const YAML::Node& node, MapBuilderParameters* p) {
  p->mapVoxelSize_ = requireScalar<double>(node, "map_voxel_size");
  loadParameters(requireNode(node, "space_carving"), &(p->carving_));
  loadParameters(requireNode(node, "scan_cropping"), &(p->cropper_));
}

void loadParameters(const YAML::Node& node, MapperParameters* p) {
  p->isBuildDenseMap_ = requireScalar<bool>(node, "is_build_dense_map");
  p->isAttemptLoopClosures_ = requireScalar<bool>(node, "is_attempt_loop_closures");
  p->minMovementBetweenMappingSteps_ = requireScalar<double>(node, "min_movement_between_mapping_steps");
  p->isIgnoreMinRefinementFitness_ = requireScalar<bool>(node, "ignore_minimum_refinement_fitness");
  p->isDumpSubmapsToFileBeforeAndAfterLoopClosures_ = requireScalar<bool>(node, "dump_submaps_to_file_before_after_lc");
  p->isPrintTimingStatistics_ = requireScalar<bool>(node, "is_print_timing_information");
  p->isRefineOdometryConstraintsBetweenSubmaps_ = requireScalar<bool>(node, "is_refine_odometry_constraints_between_submaps");
  p->isUseInitialMap_ = requireScalar<bool>(node, "is_use_map_initialization");
  p->isMergeScansIntoMap_ = requireScalar<bool>(node, "is_merge_scans_into_map");
  p->mappingBufferSize_ = requireScalar<int>(node, "mapping_buffer_size");

  const YAML::Node scanToMapRegistration = requireNode(node, "scan_to_map_registration");
  loadParameters(scanToMapRegistration, &(p->scanMatcher_));
  loadParameters(requireNode(scanToMapRegistration, "scan_processing"), &(p->scanProcessing_));

  loadParameters(requireNode(node, "map_builder"), &(p->mapBuilder_));
  loadParameters(requireNode(node, "dense_map_builder"), &(p->denseMapBuilder_));
  loadParameters(requireNode(node, "submaps"), &(p->submaps_));
  loadParameters(requireNode(node, "global_optimization"), &(p->globalOptimization_));
  loadParameters(requireNode(node, "place_recognition"), &(p->placeRecognition_));
  loadParameters(requireNode(node, "map_initializer"), &(p->mapInit_));
}

void loadParameters(const YAML::Node& node, ScanToMapRegistrationParameters* p) {
  const std::string regTypeName = requireScalar<std::string>(node, "scan_to_map_registration_type");
  p->scanToMapRegType_ = ScanToMapRegistrationStringToEnumMap.at(regTypeName);
  p->minRefinementFitness_ = requireScalar<double>(node, "min_refinement_fitness");
  loadParameters(requireNode(node, "icp"), &p->icp_);
}

void loadParameters(const YAML::Node& node, SpaceCarvingParameters* p) {
  p->voxelSize_ = requireScalar<double>(node, "voxel_size");
  p->neighborhoodRadiusDenseMap_ = requireScalar<double>(node, "neighborhood_radius_for_removal");
  p->maxRaytracingLength_ = requireScalar<double>(node, "max_raytracing_length");
  p->truncationDistance_ = requireScalar<double>(node, "truncation_distance");
  p->carveSpaceEveryNscans_ = requireScalar<int>(node, "carve_space_every_n_scans");
  p->minDotProductWithNormal_ = requireScalar<double>(node, "min_dot_product_with_normal");
}

void loadParameters(const YAML::Node& node, SlamParameters* p) {
  loadParameters(requireNode(node, "mapping"), &p->mapper_);
  loadParameters(requireNode(node, "visualization"), &p->visualization_);
  loadParameters(requireNode(node, "saving"), &p->saving_);
  loadParameters(requireNode(node, "motion_compensation"), &p->motionCompensation_);
  loadParameters(requireNode(node, "odometry"), &p->odometry_);
}

void loadParameters(const YAML::Node& node, MapInitializingParameters* p) {
  p->frameId_ = requireScalar<std::string>(node, "frame_id");
  p->pcdFilePath_ = requireScalar<std::string>(node, "pcd_file_path");
  p->isInitializeInteractively_ = requireScalar<bool>(node, "is_initialize_interactively");
  loadParameters(requireNode(node, "init_pose"), &(p->initialPose_));
}

void loadParameters(const YAML::Node& node, Eigen::Isometry3d* T) {
  const double roll = requireScalar<double>(node, "roll") * kDegToRad;
  const double pitch = requireScalar<double>(node, "pitch") * kDegToRad;
  const double yaw = requireScalar<double>(node, "yaw") * kDegToRad;
  const Eigen::Quaterniond q = fromRPY(roll, pitch, yaw).normalized();
  const Eigen::Vector3d position(requireScalar<double>(node, "x"), requireScalar<double>(node, "y"),
                                 requireScalar<double>(node, "z"));
  *T = makeTransform(position, q);
}

void loadParameters(const std::string& filename, SlamParameters* p) {
  const YAML::Node root = YAML::LoadFile(filename);
  if (root.IsNull()) {
    throw std::runtime_error("Failed to load YAML parameter file: " + filename);
  }
  loadParameters(root, p);
}

void loadParameters(const std::string& filename, MapperParameters* p) {
  const YAML::Node root = YAML::LoadFile(filename);
  if (root.IsNull()) {
    throw std::runtime_error("Failed to load YAML parameter file: " + filename);
  }
  loadParameters(requireNode(root, "mapping"), p);
}

}  // namespace io_yaml
}  // namespace o3d_slam
