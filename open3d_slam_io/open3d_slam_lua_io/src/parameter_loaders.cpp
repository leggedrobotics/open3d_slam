/*
 * parameter_loaders.cpp
 *
 *  Created on: Nov 10, 2022
 *      Author: jelavice
 */



#include "open3d_slam_lua_io/parameter_loaders.hpp"
#include "open3d_slam_lua_io/LuaLoader.hpp"
#include "open3d_slam_lua_io/helpers.hpp"

#include "open3d_slam/math.hpp"
#include "open3d_slam/output.hpp"
#include <vector>

namespace o3d_slam {
namespace io_lua {

namespace {


const double kDegToRad = M_PI / 180.0;

} //namespace

void loadParameters(const std::string &folderpath, const std::string &topLevelFileName, SlamParameters *p){
	LuaLoader loader;
	loader.setupDictionary(topLevelFileName, folderpath);
	const DictPtr &dict = loader.getDict();

	loadIfDictionaryDefined(dict,"saving", &p->saving_);
	loadIfDictionaryDefined(dict,"visualization", &p->visualization_);
	loadIfDictionaryDefined(dict,"odometry", &p->odometry_);
	loadIfDictionaryDefined(dict,"motion_compensation", &p->motionCompensation_);
	loadIfDictionaryDefined(dict,"global_optimization", &p->mapper_.globalOptimization_);
	loadIfDictionaryDefined(dict,"submap", &p->mapper_.submaps_);
	loadIfDictionaryDefined(dict,"map_builder", &p->mapper_.mapBuilder_);
	if (p->mapper_.isBuildDenseMap_){
		loadIfDictionaryDefined(dict,"dense_map_builder", &p->mapper_.denseMapBuilder_);
	}
	loadIfDictionaryDefined(dict,"mapper_localizer", &p->mapper_);
	loadIfDictionaryDefined(dict,"map_initializer", &p->mapper_.mapInit_);
	loadIfDictionaryDefined(dict,"place_recognition", &p->mapper_.placeRecognition_);
	if (!isKeyDefinedMultiLevel(dict,{"place_recognition","loop_closure_search_radius"})){
		std::cout << "Using submap size as loop closure serach radius! \n";
		p->mapper_.placeRecognition_.loopClosureSearchRadius_ = p->mapper_.submaps_.radius_; // default value
	}
}

void loadParameters(const DictPtr dict, MapperParameters *p) {
	loadIfKeyDefined(dict, "is_build_dense_map", &p->isBuildDenseMap_);
	loadIfKeyDefined(dict, "is_attempt_loop_closures", &p->isAttemptLoopClosures_);
	loadIfKeyDefined(dict, "min_movement_between_mapping_steps", &p->minMovementBetweenMappingSteps_);
	loadIfKeyDefined(dict, "dump_submaps_to_file_before_after_lc", &p->isDumpSubmapsToFileBeforeAndAfterLoopClosures_);
	loadIfKeyDefined(dict, "is_print_timing_information", &p->isPrintTimingStatistics_);
	loadIfKeyDefined(dict, "is_refine_odometry_constraints_between_submaps", &p->isRefineOdometryConstraintsBetweenSubmaps_);
	loadIfKeyDefined(dict, "is_use_map_initialization", &p->isUseInitialMap_);
	loadIfKeyDefined(dict, "is_merge_scans_into_map", &p->isMergeScansIntoMap_);
	loadIfDictionaryDefined(dict,"scan_to_map_registration", &p->scanMatcher_);
	loadIfDictionaryDefinedMultiLevel(dict,{"scan_to_map_registration","scan_processing"}, &p->scanProcessing_);
}

void loadParameters(const DictPtr dict, PlaceRecognitionParameters *p){
	loadIfKeyDefined(dict, "feature_map_normal_estimation_radius", &p->normalEstimationRadius_);
	loadIfKeyDefined(dict, "feature_voxel_size", &p->featureVoxelSize_);
	loadIfKeyDefined(dict, "feature_radius", &p->featureRadius_);
	loadIfKeyDefined(dict, "feature_knn", &p->featureKnn_);
	loadIfKeyDefined(dict, "feature_normal_knn", &p->normalKnn_);
	loadIfKeyDefined(dict, "ransac_num_iter", &p->ransacNumIter_);
	loadIfKeyDefined(dict, "ransac_probability", &p->ransacProbability_);
	loadIfKeyDefined(dict, "ransac_model_size", &p->ransacModelSize_);
	loadIfKeyDefined(dict, "ransac_max_correspondence_dist", &p->ransacMaxCorrespondenceDistance_);
	loadIfKeyDefined(dict, "ransac_correspondence_checker_distance", &p->correspondenceCheckerDistance_);
	loadIfKeyDefined(dict, "ransac_correspondence_checker_edge_length", &p->correspondenceCheckerEdgeLength_);
	loadIfKeyDefined(dict, "ransac_min_corresondence_set_size", &p->ransacMinCorrespondenceSetSize_);
	loadIfKeyDefined(dict, "max_icp_correspondence_distance", &p->maxIcpCorrespondenceDistance_);
	loadIfKeyDefined(dict, "min_icp_refinement_fitness", &p->minRefinementFitness_);
	loadIfKeyDefined(dict, "dump_aligned_place_recognitions_to_file", &p->isDumpPlaceRecognitionAlignmentsToFile_);
	loadIfKeyDefined(dict, "min_submaps_between_loop_closures", &p->minSubmapsBetweenLoopClosures_);
	loadIfKeyDefined(dict, "loop_closure_search_radius", &p->loopClosureSearchRadius_);
	loadIfDictionaryDefined(dict,"consistency_check", &p->consistencyCheck_);
}

void loadParameters(const DictPtr dict, MapInitializingParameters* p) {
	loadIfKeyDefined(dict, "frame_id", &p->frameId_);
	loadIfKeyDefined(dict, "pcd_file_path", &p->pcdFilePath_);
	loadIfKeyDefined(dict, "is_initialize_interactively", &p->isInitializeInteractively_);
	loadIfDictionaryDefined(dict,"init_pose", &p->initialPose_);
}

void loadParameters(const DictPtr dict, ScanToMapRegistrationParameters *p){
	std::string regTypeName = "";
	loadIfKeyDefined(dict, "scan_to_map_refinement_type", &regTypeName);
	p->scanToMapRegType_ = ScanToMapRegistrationStringToEnumMap.at(regTypeName);
	loadIfKeyDefined(dict, "min_refinement_fitness", &p->minRefinementFitness_);
	loadIfDictionaryDefined(dict,"icp", &p->icp_);
}


void loadParameters(const DictPtr dict, MapBuilderParameters* p) {
	loadIfKeyDefined(dict, "map_voxel_size", &p->mapVoxelSize_);
	loadIfDictionaryDefined(dict,"space_carving", &p->carving_);
	loadIfDictionaryDefined(dict,"scan_cropping", &p->cropper_);
}

void loadParameters(const DictPtr dict, SubmapParameters *p){
	loadIfKeyDefined(dict, "submap_size", &p->radius_);
	loadIfKeyDefined(dict, "min_num_range_data", &p->minNumRangeData_);
	loadIfKeyDefined(dict, "adjacency_based_revisiting_min_fitness", &p->adjacencyBasedRevisitingMinFitness_);
	loadIfKeyDefined(dict, "submaps_num_scan_overlap", &p->numScansOverlap_);
}

void loadParameters(const DictPtr dict, OdometryParameters *p){
	loadIfDictionaryDefined(dict,"scan_matching", &p->scanMatcher_);
	loadIfDictionaryDefined(dict,"scan_processing", &p->scanProcessing_);
	loadIfKeyDefined(dict, "is_publish_odometry_msgs", &p->isPublishOdometryMsgs_);
}

void loadParameters(const DictPtr dict, CloudRegistrationParameters *p){
	std::string regTypeName ="";
	loadIfKeyDefined<std::string>(dict, "cloud_registration_type", &regTypeName);
	p->regType_ = CloudRegistrationStringToEnumMap.at(regTypeName);
	loadIfDictionaryDefined(dict,"icp", &p->icp_);
}

void loadParameters(const DictPtr dict, ScanProcessingParameters *p){
	loadIfKeyDefined<double>(dict, "voxel_size", &p->voxelSize_);
	loadIfKeyDefined<double>(dict, "downsampling_ratio", &p->downSamplingRatio_);
	loadIfDictionaryDefined(dict,"scan_cropping", &p->cropper_);
}

void loadParameters(const DictPtr dict, SavingParameters *p){
	loadIfKeyDefined(dict, "save_at_mission_end", &p->isSaveAtMissionEnd_);
	loadIfKeyDefined(dict, "save_map", &p->isSaveMap_);
	loadIfKeyDefined(dict, "save_submaps", &p->isSaveSubmaps_);
}

void loadParameters(const DictPtr dict, VisualizationParameters *p){
	loadIfKeyDefined(dict, "assembled_map_voxel_size", &p->assembledMapVoxelSize_);
	loadIfKeyDefined(dict, "submaps_voxel_size", &p->submapVoxelSize_);
	loadIfKeyDefined(dict, "visualize_every_n_msec", &p->visualizeEveryNmsec_);

}

void loadParameters(const DictPtr dict, ConstantVelocityMotionCompensationParameters *p){
	loadIfKeyDefined<bool>(dict, "is_undistort_scan", &p->isUndistortInputCloud_);
	loadIfKeyDefined<bool>(dict, "is_spinning_clockwise", &p->isSpinningClockwise_);
	loadIfKeyDefined<double>(dict, "scan_duration", &p->scanDuration_);
	loadIfKeyDefined<int>(dict, "num_poses_vel_estimation", &p->numPosesVelocityEstimation_);
}

void loadParameters(const DictPtr dict, IcpParameters *p){
	loadIfKeyDefined<int>(dict, "max_n_iter", &p->maxNumIter_);
	loadIfKeyDefined<double>(dict, "max_correspondence_dist", &p->maxCorrespondenceDistance_);
	loadIfKeyDefined<int>(dict, "knn", &p->knn_);
	loadIfKeyDefined<double>(dict, "max_distance_knn", &p->maxDistanceKnn_);
}
void loadParameters(const DictPtr dict, GlobalOptimizationParameters *p){
	loadIfKeyDefined<double>(dict, "edge_prune_threshold", &p->edgePruneThreshold_);
	loadIfKeyDefined<double>(dict, "max_correspondence_distance", &p->maxCorrespondenceDistance_);
	loadIfKeyDefined<int>(dict, "reference_node", &p->referenceNode_);
	loadIfKeyDefined<double>(dict, "loop_closure_preference", &p->loopClosurePreference_);
}

void loadParameters(const DictPtr dict, PlaceRecognitionConsistencyCheckParameters *p){
	loadIfKeyDefined<double>(dict, "max_drift_pitch", &p->maxDriftPitch_);
	loadIfKeyDefined<double>(dict, "max_drift_roll", &p->maxDriftRoll_);
	loadIfKeyDefined<double>(dict, "max_drift_yaw", &p->maxDriftYaw_);
	loadIfKeyDefined<double>(dict, "max_drift_x", &p->maxDriftX_);
	loadIfKeyDefined<double>(dict, "max_drift_y", &p->maxDriftY_);
	loadIfKeyDefined<double>(dict, "max_drift_z", &p->maxDriftZ_);
	p->maxDriftRoll_ *= kDegToRad;
	p->maxDriftPitch_ *= kDegToRad;
	p->maxDriftYaw_ *= kDegToRad;
}

void loadParameters(const DictPtr dict, ScanCroppingParameters *p){
	loadIfKeyDefined<double>(dict, "cropping_radius_max", &p->croppingMaxRadius_);
	loadIfKeyDefined<double>(dict, "cropping_radius_min", &p->croppingMinRadius_);
	loadIfKeyDefined<double>(dict, "min_z", &p->croppingMinZ_);
	loadIfKeyDefined<double>(dict, "max_z", &p->croppingMaxZ_);
	loadIfKeyDefined<std::string>(dict, "cropper_type", &p->cropperName_);
}

void loadParameters(const DictPtr dict, SpaceCarvingParameters *p){
	loadIfKeyDefined<double>(dict, "voxel_size", &p->voxelSize_);
	loadIfKeyDefined<double>(dict, "max_raytracing_length", &p->maxRaytracingLength_);
	loadIfKeyDefined<double>(dict, "truncation_distance", &p->truncationDistance_);
	loadIfKeyDefined<int>(dict, "carve_space_every_n_scans", &p->carveSpaceEveryNscans_);
}

void loadParameters(const DictPtr dict, Eigen::Isometry3d* T) {
	Eigen::Isometry3d retVal = Eigen::Isometry3d::Identity();
	if (dict->HasKey("orientation")){
		DictPtr subDict = dict->GetDictionary("orientation");
		double roll(0.0),pitch(0.0),yaw(0.0);
		loadIfKeyDefined(subDict, "roll", &roll);
		loadIfKeyDefined(subDict, "pitch", &pitch);
		loadIfKeyDefined(subDict, "yaw", &yaw);
		const Eigen::Quaterniond q = fromRPY(roll,pitch,yaw).normalized();
		retVal = makeTransform(Eigen::Vector3d::Zero(), q);
	} else {
		std::cout << "[WARNING] PARAM LOAD: orientation sub-dictionary not defined \n";
	}
	if (dict->HasKey("position")){
		DictPtr subDict = dict->GetDictionary("position");
		double x(0.0),y(0.0),z(0.0);
		loadIfKeyDefined(subDict, "x", &x);
		loadIfKeyDefined(subDict, "y", &y);
		loadIfKeyDefined(subDict, "z", &z);
		retVal.translation() = Eigen::Vector3d(x,y,z);
	} else {
		std::cout << "[WARNING] PARAM LOAD: position sub-dictionary not defined \n";
	}
	*T = retVal;
}



} // namespace io_lua
} // namespace o3d_slam
