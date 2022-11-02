/*
 * Parameters.cpp
 *
 *  Created on: Sep 23, 2021
 *      Author: jelavice
 */

#include "open3d_slam/Parameters.hpp"

namespace o3d_slam {

namespace {
template<typename T, typename Ret>
void loadIfKeyDefined(const YAML::Node &node, const std::string &key, Ret *value) {
	if (node[key].IsDefined()) {
		*value = node[key].as<T>();
	} else {
		std::cout << " key " << key << " not found \n";;
	}
}

} //namespace


void loadParameters(const std::string &filename, ConstantVelocityMotionCompensationParameters *p){
	YAML::Node basenode = YAML::LoadFile(filename);
	if (basenode.IsNull()) {
		throw std::runtime_error("ConstantVelocityMotionCompensationParameters::loadParameters loading failed");
	}
	if (!basenode["motion_compensation"].IsDefined()){
		std::cout << "motion_compensation not defined \n";
		return;
	}
	loadParameters(basenode["motion_compensation"], p);
}

void loadParameters(const YAML::Node& node, ConstantVelocityMotionCompensationParameters* p) {
	p->isUndistortInputCloud_ = node["is_undistort_scan"].as<bool>();
	p->isSpinningClockwise_ = node["is_spinning_clockwise"].as<bool>();
	p->scanDuration_ = node["scan_duration"].as<double>();
	p->numPosesVelocityEstimation_ = node["num_poses_vel_estimation"].as<int>();
}

void loadParameters(const std::string &filename, SavingParameters *p){
	YAML::Node basenode = YAML::LoadFile(filename);
	if (basenode.IsNull()) {
		throw std::runtime_error("SavingParameters::loadParameters loading failed");
	}
	if (!basenode["saving_parameters"].IsDefined()){
		std::cout << "saving_parameters not defined \n";
		return;
	}
	loadParameters(basenode["saving_parameters"], p);
}

void loadParameters(const YAML::Node &node, SavingParameters *p) {
	p->isSaveAtMissionEnd_ = node["save_at_mission_end"].as<bool>();
	p->isSaveMap_ = node["save_map"].as<bool>();
	p->isSaveSubmaps_ = node["save_submaps"].as<bool>();
}

void loadParameters(const YAML::Node &node, PlaceRecognitionConsistencyCheckParameters *p){
	p->maxDriftPitch_ = node["max_drift_pitch"].as<double>() * params_internal::kDegToRad;
	p->maxDriftRoll_ =  node["max_drift_roll"].as<double>() * params_internal::kDegToRad;
	p->maxDriftYaw_ =  node["max_drift_yaw"].as<double>() * params_internal::kDegToRad;
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
	if (!basenode["visualization"].IsDefined()){
		std::cout << "visualization not defined \n";
		return;
	}
	loadParameters(basenode["visualization"], p);
}

void loadParameters(const YAML::Node &node, VisualizationParameters *p){
	p->assembledMapVoxelSize_ = node["assembled_map_voxel_size"].as<double>();
	p->submapVoxelSize_ = node["submaps_voxel_size"].as<double>();
	p->visualizeEveryNmsec_ = node["visualize_every_n_msec"].as<double>();
}

void loadParameters(const YAML::Node &n, IcpParameters *p) {
	p->knn_ = n["knn"].as<int>();
	p->maxCorrespondenceDistance_ = n["max_correspondence_dist"].as<double>();
	p->maxNumIter_ = n["max_n_iter"].as<int>();
	loadIfKeyDefined<double>(n, "max_distance_knn", &p->maxDistanceKnn_);
}

void loadParameters(const YAML::Node &node, CloudRegistrationParameters *p){
	const std::string regTypeName = node["cloud_registration_type"].as<std::string>();
	p->regType_ = CloudRegistrationNames.at(regTypeName);
	loadParameters(node["icp_parameters"], &p->icp_);

}

void loadParameters(const std::string &filename, OdometryParameters *p){
	YAML::Node basenode = YAML::LoadFile(filename);
	if (basenode.IsNull()) {
		throw std::runtime_error("Odometry::loadParameters loading failed");
	}
	if (!basenode["odometry"].IsDefined()){
		std::cout << "odometry not defined \n";
		return;
	}
	loadParameters(basenode["odometry"], p);
}
void loadParameters(const YAML::Node &node, OdometryParameters *p){
	loadParameters(node["scan_matching"], &(p->scanMatcher_) );
	loadParameters(node["scan_processing"], &(p->scanProcessing_) );
	loadIfKeyDefined<bool>(node,"is_publish_odometry_msgs", &p->isPublishOdometryMsgs_);
}

void loadParameters(const YAML::Node &node, ScanProcessingParameters *p){
	p->voxelSize_ = node["voxel_size"].as<double>();
	p->downSamplingRatio_ = node["downsampling_ratio"].as<double>();
	loadParameters(node["scan_cropping"], &(p->cropper_));
}

void loadParameters(const YAML::Node &node, ScanCroppingParameters *p){
	p->croppingMaxRadius_ = node["cropping_radius_max"].as<double>();
	p->croppingMinRadius_ = node["cropping_radius_min"].as<double>();
	p->croppingMinZ_ = node["min_z"].as<double>();
	p->croppingMaxZ_ = node["max_z"].as<double>();
	p->cropperName_ = node["cropper_type"].as<std::string>();
}

void loadParameters(const YAML::Node &node, SubmapParameters *p){
	p->radius_ = node["size"].as<double>();
	p->minNumRangeData_ = node["min_num_range_data"].as<int>();
	p->adjacencyBasedRevisitingMinFitness_ = node["adjacency_based_revisiting_min_fitness"].as<double>();
}

void loadParameters(const YAML::Node& node, MapBuilderParameters* p) {
	p->mapVoxelSize_ = node["map_voxel_size"].as<double>();
	loadParameters(node["space_carving"], &(p->carving_));
	loadParameters(node["scan_cropping"], &(p->cropper_));
}

void loadParameters(const std::string &filename, MapperParameters *p){
	YAML::Node basenode = YAML::LoadFile(filename);
	if (basenode.IsNull()) {
		throw std::runtime_error("MapperParameters::loadParameters loading failed");
	}
	if (!basenode["mapping"].IsDefined()){
		std::cout << "mapping not defined \n";
		return;
	}
	loadParameters(basenode["mapping"], p);
}


void loadParameters(const YAML::Node &node, MapperParameters *p) {
	p->isBuildDenseMap_ = node["is_build_dense_map"].as<bool>();
	p->isAttemptLoopClosures_ = node["is_attempt_loop_closures"].as<bool>();
	p->minMovementBetweenMappingSteps_ = node["min_movement_between_mapping_steps"].as<double>();
	p->numScansOverlap_ = node["submaps_num_scan_overlap"].as<int>();
	p->isDumpSubmapsToFileBeforeAndAfterLoopClosures_ = node["dump_submaps_to_file_before_after_lc"].as<bool>();
	p->isPrintTimingStatistics_ = node["is_print_timing_information"].as<bool>();
	p->isRefineOdometryConstraintsBetweenSubmaps_ = node["is_refine_odometry_constraints_between_submaps"].as<bool>();
	p->isUseInitialMap_ = node["is_use_map_initialization"].as<bool>();
	p->isMergeScansIntoMap_ = node["is_merge_scans_into_map"].as<bool>();
	loadParameters(node["scan_to_map_refinement"],&(p->scanMatcher_));
	loadParameters(node["scan_to_map_refinement"]["scan_processing"], &(p->scanProcessing_));

	if (p->isBuildDenseMap_) {
		loadParameters(node["dense_map_builder"], &(p->denseMapBuilder_));
	}
	loadParameters(node["map_builder"], &(p->mapBuilder_));
	loadParameters(node["submaps"], &(p->submaps_));
	loadParameters(node["global_optimization"], &(p->globalOptimization_));
	loadParameters(node["place_recognition"], &(p->placeRecognition_));
}

void loadParameters(const YAML::Node &node, ScanToMapRegistrationParameters *p){
	const std::string regTypeName = node["scan_to_map_refinement_type"].as<std::string>();
	p->scanToMapRegType_ = ScanToMapRegistrationNames.at(regTypeName);
	p->minRefinementFitness_ = node["min_refinement_fitness"].as<double>();
	loadParameters(node["icp_parameters"], &p->icp_);
}

void loadParameters(const YAML::Node &node, SpaceCarvingParameters *p){
  if (node["voxel_size"].IsDefined()){
    p->voxelSize_ = node["voxel_size"].as<double>();
  }
  if (node["neigborhood_radius_for_removal"].IsDefined()){
      p->neighborhoodRadiusDenseMap_ = node["neigborhood_radius_for_removal"].as<double>();
    }
	p->maxRaytracingLength_ = node["max_raytracing_length"].as<double>();
	p->truncationDistance_ = node["truncation_distance"].as<double>();
	p->carveSpaceEveryNscans_ = node["carve_space_every_n_scans"].as<int>();
	p->minDotProductWithNormal_ = node["min_dot_product_with_normal"].as<double>();
}

} // namespace o3d_slam

