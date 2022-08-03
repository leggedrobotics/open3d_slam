/*
 * Parameters.cpp
 *
 *  Created on: Sep 23, 2021
 *      Author: jelavice
 */

#include "open3d_slam/Parameters.hpp"

namespace o3d_slam {

void loadParameters(const std::string &filename, ConstantVelocityMotionCompensationParameters *p){
	YAML::Node basenode = YAML::LoadFile(filename);
		if (basenode.IsNull()) {
			throw std::runtime_error(
					"Motion compensation params::loadParameters loading failed");
		}
		if (basenode["motion_compensation"].IsDefined()){
			loadParameters(basenode["motion_compensation"], p);
		}
}
void loadParameters(const YAML::Node &node, ConstantVelocityMotionCompensationParameters *p){
	p->isUndistortInputCloud_ = node["is_undistort_scan"].as<bool>();
	p->isSpinningClockwise_ = node["is_spinning_clockwise"].as<bool>();
	p->scanDuration_ = node["scan_duration"].as<double>();
	p->numPosesVelocityEstimation_ = node["num_poses_vel_estimation"].as<int>();
}

void loadParameters(const std::string &filename, SavingParameters *p) {
	YAML::Node basenode = YAML::LoadFile(filename);
	if (basenode.IsNull()) {
		throw std::runtime_error(
				"MapSavingParameters::loadParameters loading failed");
	}
	loadParameters(basenode["saving_parameters"], p);
}
void loadParameters(const YAML::Node &node, SavingParameters *p) {
	p->isSaveAtMissionEnd_ = node["save_at_mission_end"].as<bool>();
	p->isSaveMap_ = node["save_map"].as<bool>();
	p->isSaveSubmaps_ = node["save_submaps"].as<bool>();
}

void loadParameters(const std::string &filename,
		PlaceRecognitionConsistencyCheckParameters *p) {
	YAML::Node basenode = YAML::LoadFile(filename);
	if (basenode.IsNull()) {
		throw std::runtime_error(
				"PlaceRecognitionParams::loadParameters loading failed");
	}
	loadParameters(basenode["consistency_check"], p);
}
void loadParameters(const YAML::Node &node, PlaceRecognitionConsistencyCheckParameters *p){
	p->maxDriftPitch_ = node["max_drift_pitch"].as<double>() * params_internal::kDegToRad;
	p->maxDriftRoll_ =  node["max_drift_roll"].as<double>() * params_internal::kDegToRad;
	p->maxDriftYaw_ =  node["max_drift_yaw"].as<double>() * params_internal::kDegToRad;
}

void loadParameters(const std::string &filename, PlaceRecognitionParameters *p){
	YAML::Node basenode = YAML::LoadFile(filename);
	if (basenode.IsNull()) {
		throw std::runtime_error("PlaceRecognitionParams::loadParameters loading failed");
	}
	loadParameters(basenode["place_recognition"], p);
}
void loadParameters(const YAML::Node &node, PlaceRecognitionParameters *p){

	p->normalEstimationRadius_ = node["feature_map_normal_estimation_radius"].as<double>();
	p->featureVoxelSize_ = node["feature_voxel_size"].as<double>();
	p->featureRadius_ = node["feature_radius"].as<double>();
	p->featureKnn_ = node["feature_knn"].as<int>();
	p->normalKnn_ = node["feature_normal_knn"].as<int>();
	p->ransacNumIter_ = node["ransac_num_iter"].as<int>();
	p->ransacProbability_ = node["ransac_probability"].as<double>();
	p->ransacModelSize_ = node["ransac_model_size"].as<int>();
	p->ransacMaxCorrespondenceDistance_ = node["ransac_max_correspondence_dist"].as<double>();
	p->correspondenceCheckerDistance_ = node["ransac_correspondence_checker_distance"].as<double>();
	p->correspondenceCheckerEdgeLength_ = node["ransac_correspondence_checker_edge_length"].as<double>();
	p->ransacMinCorrespondenceSetSize_ = node["ransac_min_corresondence_set_size"].as<int>();
	p->maxIcpCorrespondenceDistance_ = node["max_icp_correspondence_distance"].as<double>();
	p->minRefinementFitness_ = node["min_icp_refinement_fitness"].as<double>();
	p->isDumpPlaceRecognitionAlignmentsToFile_ = node["dump_aligned_place_recognitions_to_file"].as<bool>();
	loadParameters(node["consistency_check"], &(p->consistencyCheck_));
}

void loadParameters(const std::string &filename, GlobalOptimizationParameters *p){
	YAML::Node basenode = YAML::LoadFile(filename);
	if (basenode.IsNull()) {
		throw std::runtime_error("GlobalOptimizationParameters::loadParameters loading failed");
	}
	loadParameters(basenode["global_optimization"], p);
}
void loadParameters(const YAML::Node &node, GlobalOptimizationParameters *p){
	p->edgePruneThreshold_ = node["edge_prune_threshold"].as<double>();
	p->loopClosurePreference_ = node["loop_closure_preference"].as<double>();
	p->maxCorrespondenceDistance_ = node["max_correspondence_distance"].as<double>();
	p->referenceNode_ = node["reference_node"].as<int>();
}


void loadParameters(const std::string &filename, VisualizationParameters *p){
	YAML::Node basenode = YAML::LoadFile(filename);
	if (basenode.IsNull()) {
		throw std::runtime_error("VisualizationParameters::loadParameters loading failed");
	}
	loadParameters(basenode["visualization"], p);
}
void loadParameters(const YAML::Node &node, VisualizationParameters *p){
	p->assembledMapVoxelSize_ = node["assembled_map_voxel_size"].as<double>();
	p->submapVoxelSize_ = node["submaps_voxel_size"].as<double>();
	p->visualizeEveryNmsec_ = node["visualize_every_n_msec"].as<double>();
}


void loadParameters(const std::string &filename, IcpParameters *p) {
	YAML::Node basenode = YAML::LoadFile(filename);
	if (basenode.IsNull()) {
		throw std::runtime_error("IcpParameters::loadParameters loading failed");
	}
	loadParameters(basenode["odometry"], p);
}

void loadParameters(const YAML::Node &n, IcpParameters *p) {
	p->icpObjective_ = IcpObjectiveNames.at(n["icp_objective"].as<std::string>());
	p->kNNnormalEstimation_ = n["knn_normal_estimation"].as<int>();
	p->maxCorrespondenceDistance_ = n["max_correspondence_dist"].as<double>();
	p->maxNumIter_ = n["max_n_iter"].as<int>();
}

void loadParameters(const std::string &filename, OdometryParameters *p){
	YAML::Node basenode = YAML::LoadFile(filename);
	if (basenode.IsNull()) {
		throw std::runtime_error("Odometry::loadParameters loading failed");
	}
	loadParameters(basenode["odometry"], p);
}
void loadParameters(const YAML::Node &node, OdometryParameters *p){
	loadParameters(node["scan_matching"], &(p->scanMatcher_) );
	loadParameters(node["scan_processing"], &(p->scanProcessing_) );
	p->minRefinementFitness_ = node["min_refinement_fitness"].as<double>();
	if (node["is_publish_odometry_msgs"].IsDefined()){
	    p->isPublishOdometryMsgs_ = node["is_publish_odometry_msgs"].as<bool>();
	}
}

void loadParameters(const std::string &filename, ScanProcessingParameters *p){
	YAML::Node basenode = YAML::LoadFile(filename);
	if (basenode.IsNull()) {
		throw std::runtime_error("ScanProcessingParameters::loadParameters loading failed");
	}
	loadParameters(basenode["scan_processing"], p);
}
void loadParameters(const YAML::Node &node, ScanProcessingParameters *p){
	p->voxelSize_ = node["voxel_size"].as<double>();
	p->downSamplingRatio_ = node["downsampling_ratio"].as<double>();
	loadParameters(node["scan_cropping"], &(p->cropper_));
}

void loadParameters(const std::string &filename, ScanCroppingParameters *p){
	YAML::Node basenode = YAML::LoadFile(filename);
	if (basenode.IsNull()) {
		throw std::runtime_error("ScanCroppingParameters::loadParameters loading failed");
	}
	loadParameters(basenode["scan_cropping"], p);
}
void loadParameters(const YAML::Node &node, ScanCroppingParameters *p){
	p->croppingMaxRadius_ = node["cropping_radius_max"].as<double>();
	p->croppingMinRadius_ = node["cropping_radius_min"].as<double>();
	p->croppingMinZ_ = node["min_z"].as<double>();
	p->croppingMaxZ_ = node["max_z"].as<double>();
	p->cropperName_ = node["cropper_type"].as<std::string>();
}

void loadParameters(const std::string &filename, SubmapParameters *p){
	YAML::Node basenode = YAML::LoadFile(filename);
	if (basenode.IsNull()) {
		throw std::runtime_error("SubmapParameters::loadParameters loading failed");
	}
	loadParameters(basenode["submaps"], p);
}
void loadParameters(const YAML::Node &node, SubmapParameters *p){
	p->radius_ = node["size"].as<double>();
	p->minNumRangeData_ = node["min_num_range_data"].as<int>();
	p->adjacencyBasedRevisitingMinFitness_ = node["adjacency_based_revisiting_min_fitness"].as<double>();
}

void loadParameters(const std::string &filename, MapBuilderParameters *p) {
	YAML::Node basenode = YAML::LoadFile(filename);
	if (basenode.IsNull()) {
		throw std::runtime_error("MapBuilderParameters::loadParameters loading failed");
	}
	loadParameters(basenode["map_builder"], p);
}
void loadParameters(const YAML::Node &node, MapBuilderParameters *p) {
	p->mapVoxelSize_ = node["map_voxel_size"].as<double>();
	loadParameters(node["space_carving"], &(p->carving_));
	loadParameters(node["scan_cropping"], &(p->cropper_));
}
void loadParameters(const YAML::Node &node, MapInitializationParameters *p) {
	p->maxTranslationError_ = node["max_translation_error"].as<double>();
	p->maxAngleError_ = node["max_angle_error"].as<double>();
	p->initializeMap_ = node["initialize_map"].as<bool>();
}


void loadParameters(const std::string &filename, MapperParameters *p) {
	YAML::Node basenode = YAML::LoadFile(filename);

	if (basenode.IsNull()) {
		throw std::runtime_error("MapperParameters::loadParameters loading failed");
	}

	loadParameters(basenode["mapping"], p);
}
void loadParameters(const YAML::Node &node, MapperParameters *p) {
	p->isBuildDenseMap_ = node["is_build_dense_map"].as<bool>();
	p->isAttemptLoopClosures_ = node["is_attempt_loop_closures"].as<bool>();
	p->minMovementBetweenMappingSteps_ = node["min_movement_between_mapping_steps"].as<double>();
	p->minRefinementFitness_ = node["scan_to_map_refinement"]["min_refinement_fitness"].as<double>();
	p->numScansOverlap_ = node["submaps_num_scan_overlap"].as<int>();
	p->isDumpSubmapsToFileBeforeAndAfterLoopClosures_ = node["dump_submaps_to_file_before_after_lc"].as<bool>();
	p->isPrintTimingStatistics_ = node["is_print_timing_information"].as<bool>();
	p->isRefineOdometryConstraintsBetweenSubmaps_ = node["is_refine_odometry_constraints_between_submaps"].as<bool>();
	loadParameters(node["scan_to_map_refinement"]["scan_matching"],&(p->scanMatcher_));
	loadParameters(node["scan_to_map_refinement"]["scan_processing"],&(p->scanProcessing_));
	loadParameters(node["map_intialization"],&(p->mapInitialization_));
	if(p->isBuildDenseMap_){
		loadParameters(node["dense_map_builder"],&(p->denseMapBuilder_));
	}
	loadParameters(node["map_builder"], &(p->mapBuilder_));
	loadParameters(node["submaps"], &(p->submaps_));
	loadParameters(node["global_optimization"], &(p->globalOptimization_));
	loadParameters(node["place_recognition"], &(p->placeRecognition_));
}

void loadParameters(const YAML::Node &n, SpaceCarvingParameters *p){
  if (n["voxel_size"].IsDefined()){
    p->voxelSize_ = n["voxel_size"].as<double>();
  }
  if (n["neigborhood_radius_for_removal"].IsDefined()){
      p->neighborhoodRadiusDenseMap_ = n["neigborhood_radius_for_removal"].as<double>();
    }
	p->maxRaytracingLength_ = n["max_raytracing_length"].as<double>();
	p->truncationDistance_ = n["truncation_distance"].as<double>();
	p->carveSpaceEveryNscans_ = n["carve_space_every_n_scans"].as<int>();
	p->minDotProductWithNormal_ = n["min_dot_product_with_normal"].as<double>();
}
void loadParameters(const std::string &filename, SpaceCarvingParameters *p){
	YAML::Node basenode = YAML::LoadFile(filename);
	if (basenode.IsNull()) {
		throw std::runtime_error("SpaceCarving::loadParameters loading failed");
	}
	loadParameters(basenode["space_carving"], p);
}

} // namespace o3d_slam

