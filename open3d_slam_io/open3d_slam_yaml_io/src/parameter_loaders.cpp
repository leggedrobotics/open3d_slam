/*
 * parameter_loaders.cpp
 *
 *  Created on: Nov 10, 2022
 *      Author: jelavice
 */



#include "open3d_slam_yaml_io/parameter_loaders.hpp"
#include "open3d_slam/math.hpp"
#include "open3d_slam/output.hpp"


namespace o3d_slam {
namespace io_yaml {

namespace {
template<typename T, typename Ret>
void loadIfKeyDefined(const YAML::Node &node, const std::string &key, Ret *value) {
	if (node[key].IsDefined()) {
		*value = node[key].as<T>();
	} else {
		std::cout << " key " << key << " not found \n";;
	}
}

const double kDegToRad = M_PI / 180.0;

} //namespace


void loadParameters(const YAML::Node& node, ConstantVelocityMotionCompensationParameters* p) {
	p->isUndistortInputCloud_ = node["is_undistort_scan"].as<bool>();
	p->isSpinningClockwise_ = node["is_spinning_clockwise"].as<bool>();
	p->scanDuration_ = node["scan_duration"].as<double>();
	p->numPosesVelocityEstimation_ = node["num_poses_vel_estimation"].as<int>();
}

void loadParameters(const YAML::Node &node, SavingParameters *p) {
	p->isSaveAtMissionEnd_ = node["save_at_mission_end"].as<bool>();
	p->isSaveMap_ = node["save_map"].as<bool>();
	p->isSaveSubmaps_ = node["save_submaps"].as<bool>();
	if (node["save_dense_submaps"].IsDefined()){
		p->isSaveDenseSubmaps_ = node["save_dense_submaps"].as<bool>();
	}
}

void loadParameters(const YAML::Node &node, PlaceRecognitionConsistencyCheckParameters *p){
	p->maxDriftPitch_ = node["max_drift_pitch"].as<double>() * params_internal::kDegToRad;
	p->maxDriftRoll_ =  node["max_drift_roll"].as<double>() * params_internal::kDegToRad;
	p->maxDriftYaw_ =  node["max_drift_yaw"].as<double>() * params_internal::kDegToRad;
	loadIfKeyDefined<double>(node, "max_drift_x", &p->maxDriftX_);
	loadIfKeyDefined<double>(node, "max_drift_y", &p->maxDriftY_);
	loadIfKeyDefined<double>(node, "max_drift_z", &p->maxDriftZ_);
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
	loadIfKeyDefined<int>(node, "min_submaps_between_loop_closures", &p->minSubmapsBetweenLoopClosures_);
	loadIfKeyDefined<double>(node, "loop_closure_serach_radius", &p->loopClosureSearchRadius_);

	loadParameters(node["consistency_check"], &(p->consistencyCheck_));
}

void loadParameters(const YAML::Node &node, GlobalOptimizationParameters *p){
	p->edgePruneThreshold_ = node["edge_prune_threshold"].as<double>();
	p->loopClosurePreference_ = node["loop_closure_preference"].as<double>();
	p->maxCorrespondenceDistance_ = node["max_correspondence_distance"].as<double>();
	p->referenceNode_ = node["reference_node"].as<int>();
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
	p->regType_ = CloudRegistrationStringToEnumMap.at(regTypeName);
	loadParameters(node["icp_parameters"], &p->icp_);
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
	p->numScansOverlap_ = node["submaps_num_scan_overlap"].as<int>();
}

void loadParameters(const YAML::Node& node, MapBuilderParameters* p) {
	p->mapVoxelSize_ = node["map_voxel_size"].as<double>();
	loadParameters(node["space_carving"], &(p->carving_));
	loadParameters(node["scan_cropping"], &(p->cropper_));
}

void loadParameters(const YAML::Node &node, MapperParameters *p) {
	p->isBuildDenseMap_ = node["is_build_dense_map"].as<bool>();
	p->isAttemptLoopClosures_ = node["is_attempt_loop_closures"].as<bool>();
	p->minMovementBetweenMappingSteps_ = node["min_movement_between_mapping_steps"].as<double>();
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
	if (!node["place_recognition"]["loop_closure_serach_radius"].IsDefined()){
		std::cout << "Using submap size as loop closure serach radius! \n";
		p->placeRecognition_.loopClosureSearchRadius_ = p->submaps_.radius_; // default value
	}
	loadParameters(node["map_intializer"], &(p->mapInit_));
}

void loadParameters(const YAML::Node &node, ScanToMapRegistrationParameters *p){
	const std::string regTypeName = node["scan_to_map_refinement_type"].as<std::string>();
	p->scanToMapRegType_ = ScanToMapRegistrationStringToEnumMap.at(regTypeName);
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



void loadParameters(const YAML::Node &node, SlamParameters *p){
	if (node["mapping"].IsDefined()){
		loadParameters(node["mapping"], &p->mapper_);
	} else {
		std::cout << "mapping parameters not defined \n";
	}
	if (node["visualization"].IsDefined()){
		loadParameters(node["visualization"], &p->visualization_);
	} else {
		std::cout << "visualization parameters not defined \n";
	}
	if (node["saving_parameters"].IsDefined()){
		loadParameters(node["saving_parameters"], &p->saving_);
	} else {
		std::cout << "saving_parameters not defined \n";
	}
	if (node["motion_compensation"].IsDefined()){
		loadParameters(node["motion_compensation"], &p->motionCompensation_);
	} else {
		std::cout << "motion_compensation parameters not defined \n";
	}
	if (node["odometry"].IsDefined()){
		loadParameters(node["odometry"], &p->odometry_);
	} else {
		std::cout << "odometry parameters not defined \n";
	}

}

void loadParameters(const YAML::Node& node, MapInitializingParameters* p) {
	p->frameId_ = node["frame_id"].as<std::string>();
	p->pcdFilePath_ = node["pcd_file_path"].as<std::string>();
	p->isInitializeInteractively_ = node["is_initialize_interactively"].as<bool>();
	if (node["init_pose"].IsDefined()) {
		loadParameters(node["init_pose"], &(p->initialPose_));
	}
}

void loadParameters(const YAML::Node& node, Eigen::Isometry3d* T) {
	Eigen::Isometry3d retVal = Eigen::Isometry3d::Identity();
	if (node["orientation"].IsDefined()){
		const auto &n = node["orientation"];
		double roll(0.0),pitch(0.0),yaw(0.0);
		if (n["roll"].IsDefined()){
		  roll = n["roll"].as<double>() * kDegToRad;
		}
		if (n["pitch"].IsDefined()) {
			pitch = n["pitch"].as<double>() * kDegToRad;
		}
		if (n["yaw"].IsDefined()){
		  yaw = n["yaw"].as<double>() * kDegToRad;
		}
		const Eigen::Quaterniond q = fromRPY(roll,pitch,yaw).normalized();
		retVal = makeTransform(Eigen::Vector3d::Zero(), q);
	}
	if (node["position"].IsDefined()){
		const auto &n = node["position"];
		if (n["x"].IsDefined()){
		  retVal.translation().x() = n["x"].as<float>();
		}
		if (n["y"].IsDefined()) {
			retVal.translation().y() = n["y"].as<float>();
		}
		if (n["z"].IsDefined()){
			retVal.translation().z() = n["z"].as<float>();
		}

	}
	std::cout << "here \n";
	std::cout << asString(retVal) << "\n";
	*T = retVal;
}

void loadParameters(const std::string &filename, SlamParameters *p){
	YAML::Node basenode = YAML::LoadFile(filename);
	if (basenode.IsNull()) {
		throw std::runtime_error("SlamParameters::loadParameters loading failed");
	}
	loadParameters(basenode, p);
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


} // namespace io_yaml
} // namespace o3d_slam
