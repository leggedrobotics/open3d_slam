/*
 * LuaLoader.cpp
 *
 *  Created on: Nov 11, 2022
 *      Author: jelavice
 */

#include "open3d_slam_lua_io/LuaLoader.hpp"
#include "open3d_slam/math.hpp"
#include "open3d_slam/output.hpp"
#include <vector>
#include <iostream>
#include <stack>
#include <boost/filesystem.hpp>

namespace o3d_slam {
namespace io_lua {

namespace {
std::stack<std::string> luaParamListNameStack;
const double kDegToRad = M_PI / 180.0;
std::string rootParamName = "o3d_params";
const std::string paramNameDelimiter = ".";
std::string extractPrefix(const std::stack<std::string> &S) {
	std::stack<std::string> inverse;
	std::stack<std::string> copy = S;
	while (!copy.empty()) {
		std::string key = copy.top();
		inverse.push(key);
		copy.pop();
	}
	std::string retVal;
	while (!inverse.empty()) {
		std::string key = inverse.top();
		const std::string prefix = retVal.empty() ? "":paramNameDelimiter;
		retVal += prefix + key;
		inverse.pop();
	}
	return retVal;
}

std::string parseTopLevelName(const std::string &s){
	std::size_t found = s.rfind("return");
	if (found == std::string::npos) {
		throw std::runtime_error(
				"your configuration file must contain return statement where you return the parameter structure. See examples");
	}
	std::string str = s.substr(found+7);
	str.erase(std::remove(str.begin(), str.end(), '\n'), str.cend());
	return str;
}

std::vector<std::string> includeDirectoriesRecursively(const std::string &folderPath){
	std::vector<std::string> retVal;
	using namespace boost::filesystem;
	    recursive_directory_iterator dir( folderPath), end;
	    while (dir != end)
	    {
	    	if (is_directory(*dir)){
	        retVal.push_back(dir->path().c_str());
	        // do other stuff here.
	        std::cout << "recursively including folder: " << retVal.back() << " \n";
	    	}
	        ++dir;
	    }
	    if (retVal.empty()){
	    	std::cout << "not including any additional directories \n";
	    }
	return retVal;
}

} // namespace

using namespace lua_dict;

void LuaLoader::setupDictionary(const std::string &topLevelFileName, const std::string &folderPath) {
	const std::vector<std::string> extraDirs = includeDirectoriesRecursively(folderPath);
	std::vector<std::string> paths( { folderPath });
	paths.insert(paths.end(), extraDirs.begin(), extraDirs.end());
	folderPaths_ = paths;
	auto fileResolver = std::make_unique<ConfigurationFileResolver>(folderPaths_);
	const std::string fullContent = fileResolver->GetFileContentOrDie(topLevelFileName);
	const std::string fullPath = fileResolver->GetFullPathOrDie(topLevelFileName);
	const std::string structName = parseTopLevelName(fullContent);
	rootParamName = structName;
//	dict_ = std::make_shared<LuaParameterDictionary>(fullContent, std::move(fileResolver));
	dict_ = LuaParameterDictionary::NonReferenceCounted(fullContent, std::move(fileResolver));
	topFileName_ = topLevelFileName;
	std::cout << "Top level param struct resolved to be: " << structName << std::endl;
	std::cout << "Lua loader resolved full path, loaded from: " << fullPath << std::endl;
}

const DictPtr& LuaLoader::getDict() const {
	return dict_;
}
void LuaLoader::buildLuaParamList() {
	luaParamListNameStack = std::stack<std::string>(); // empty stack
	luaParamListNameStack.push(rootParamName);
	auto fileResolver = std::make_unique<ConfigurationFileResolver>(folderPaths_);
	const std::string fullContent = fileResolver->GetFileContentOrDie(topFileName_);
	const std::string fullPath = fileResolver->GetFullPathOrDie(topFileName_);
	auto dict = LuaParameterDictionary::NonReferenceCounted(fullContent, std::move(fileResolver));
	luaParamList_.clear();
	treeTraversal(dict);
}
void LuaLoader::treeTraversal(const DictPtr &dict) {
	auto keys = dict->GetKeys();
	for (const auto &key : keys) {
		if (dict->IsTable(key)) {
			auto subDict = dict->GetDictionary(key);
			luaParamListNameStack.push(key);
			treeTraversal(subDict);
		} else {
			auto name = extractPrefix(luaParamListNameStack) + paramNameDelimiter + key;
			luaParamList_.insert(name);
//			std::cout << "Name: " << name << std::endl;
		}
	}
	luaParamListNameStack.pop();
}

void LuaLoader::incrementRefCount(const std::string &key){
		std::string name = extractPrefix(loadingNameStack_) +paramNameDelimiter + key;
		auto search = loadingParamCount_.find(name);
		if (search == loadingParamCount_.end()){
			loadingParamCount_[name] = 1;
		} else {
			loadingParamCount_[name]++; // todo extra search
		}
	}

void LuaLoader::loadIntIfKeyDefined(const DictPtr &dict, const std::string &key, int *value) {
	if (dict->HasKey(key)) {
		*value = dict->GetInt(key);
			incrementRefCount(key);
	} else {
		std::cout << " \033[1;33m[WARNING]:\033[0m PARAM LOAD INT: key " << key << " not found \n";
		const std::string keysAvailable = getKeysAsStringCsv(*dict);
		std::cout << "	keys availalbe: " << keysAvailable << "\n\n";
	}
}
void LuaLoader::loadDoubleIfKeyDefined(const DictPtr &dict, const std::string &key, double *value) {
	if (dict->HasKey(key)) {
		*value = dict->GetDouble(key);
		incrementRefCount(key);
	} else {
		std::cout << " \033[1;33m[WARNING]:\033[0m PARAM LOAD DOUBLE: key " << key << " not found \n";
		const std::string keysAvailable = getKeysAsStringCsv(*dict);
		std::cout << "	keys availalbe: " << keysAvailable << "\n\n";
	}
}
void LuaLoader::loadStringIfKeyDefined(const DictPtr &dict, const std::string &key, std::string *value) {
	if (dict->HasKey(key)) {
		*value = dict->GetString(key);
			incrementRefCount(key);
	} else {
		std::cout << " \033[1;33m[WARNING]:\033[0m PARAM LOAD STRING: key " << key << " not found \n";
		const std::string keysAvailable = getKeysAsStringCsv(*dict);
		std::cout << "	keys availalbe: " << keysAvailable << "\n\n";
	}
}
void LuaLoader::loadBoolIfKeyDefined(const DictPtr &dict, const std::string &key, bool *value) {
	if (dict->HasKey(key)) {
		*value = dict->GetBool(key);
			incrementRefCount(key);
	} else {
		std::cout << " \033[1;33m[WARNING]:\033[0m PARAM LOAD BOOL: key " << key << " not found \n";
		const std::string keysAvailable = getKeysAsStringCsv(*dict);
		std::cout << "	keys availalbe: " << keysAvailable << "\n\n";
	}
}

bool LuaLoader::isLoadingOkay() {
  buildLuaParamList();
  bool res = true;
  for (auto it = loadingParamCount_.begin(); it != loadingParamCount_.end(); it++)
  {
      if (it->second > 1){
      	std::cout << "\033[1;33m[WARNING]: entry \033[0m " << it->first << "has been loaded: " << it->second << " times\n";
      	res = false;
      }
  }
  for (auto it = luaParamList_.begin(); it != 	luaParamList_.end(); ++it){
  	auto search = loadingParamCount_.find(*it);
  	if (search == loadingParamCount_.end()){
  		std::cout << "\033[1;33m[WARNING]: entry \033[0m " << *it << " was not loaded \n";
  		res = false;
  	}
  }

  return res;
}


/*
 *//////////////////////////////////
/////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
 ////////////////////////////////

void LuaLoader::loadParameters(const std::string &folderpath, const std::string &topLevelFileName, SlamParameters *p){
	loadingNameStack_ = std::stack<std::string>(); // empty stack
	loadingNameStack_.push(rootParamName);
	const DictPtr &dict = dict_;
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
	std::cout << "Loading complete! \n";
}

void LuaLoader::loadParameters(const DictPtr dict, MapperParameters *p) {
	loadBoolIfKeyDefined(dict, "is_build_dense_map", &p->isBuildDenseMap_);
	loadBoolIfKeyDefined(dict, "is_attempt_loop_closures", &p->isAttemptLoopClosures_);
	loadBoolIfKeyDefined(dict, "dump_submaps_to_file_before_after_lc", &p->isDumpSubmapsToFileBeforeAndAfterLoopClosures_);
	loadBoolIfKeyDefined(dict, "is_print_timing_information", &p->isPrintTimingStatistics_);
	loadBoolIfKeyDefined(dict, "is_refine_odometry_constraints_between_submaps", &p->isRefineOdometryConstraintsBetweenSubmaps_);
	loadBoolIfKeyDefined(dict, "is_use_map_initialization", &p->isUseInitialMap_);
	loadBoolIfKeyDefined(dict, "is_merge_scans_into_map", &p->isMergeScansIntoMap_);

	loadDoubleIfKeyDefined(dict, "min_movement_between_mapping_steps", &p->minMovementBetweenMappingSteps_);


	loadIfDictionaryDefined(dict,"scan_to_map_registration", &p->scanMatcher_);
	loadIfDictionaryDefinedMultiLevel(dict,{"scan_to_map_registration","scan_processing"}, &p->scanProcessing_);
}

void LuaLoader::loadParameters(const DictPtr dict, PlaceRecognitionParameters *p){
	loadDoubleIfKeyDefined(dict, "feature_map_normal_estimation_radius", &p->normalEstimationRadius_);
	loadDoubleIfKeyDefined(dict, "feature_voxel_size", &p->featureVoxelSize_);
	loadDoubleIfKeyDefined(dict, "feature_radius", &p->featureRadius_);
	loadDoubleIfKeyDefined(dict, "ransac_probability", &p->ransacProbability_);
	loadDoubleIfKeyDefined(dict, "max_icp_correspondence_distance", &p->maxIcpCorrespondenceDistance_);
	loadDoubleIfKeyDefined(dict, "min_icp_refinement_fitness", &p->minRefinementFitness_);
	loadDoubleIfKeyDefined(dict, "ransac_max_correspondence_dist", &p->ransacMaxCorrespondenceDistance_);
	loadDoubleIfKeyDefined(dict, "ransac_correspondence_checker_distance", &p->correspondenceCheckerDistance_);
	loadDoubleIfKeyDefined(dict, "ransac_correspondence_checker_edge_length", &p->correspondenceCheckerEdgeLength_);
	loadDoubleIfKeyDefined(dict, "loop_closure_search_radius", &p->loopClosureSearchRadius_);

	loadIntIfKeyDefined(dict, "feature_knn", &p->featureKnn_);
	loadIntIfKeyDefined(dict, "feature_normal_knn", &p->normalKnn_);
	loadIntIfKeyDefined(dict, "ransac_num_iter", &p->ransacNumIter_);
	loadIntIfKeyDefined(dict, "ransac_model_size", &p->ransacModelSize_);
	loadIntIfKeyDefined(dict, "ransac_min_corresondence_set_size", &p->ransacMinCorrespondenceSetSize_);
	loadIntIfKeyDefined(dict, "min_submaps_between_loop_closures", &p->minSubmapsBetweenLoopClosures_);

	loadBoolIfKeyDefined(dict, "dump_aligned_place_recognitions_to_file", &p->isDumpPlaceRecognitionAlignmentsToFile_);

	loadIfDictionaryDefined(dict,"consistency_check", &p->consistencyCheck_);
}

void LuaLoader::loadParameters(const DictPtr dict, MapInitializingParameters* p) {
	loadStringIfKeyDefined(dict, "frame_id", &p->frameId_);
	loadStringIfKeyDefined(dict, "pcd_file_path", &p->pcdFilePath_);
	loadBoolIfKeyDefined(dict, "is_initialize_interactively", &p->isInitializeInteractively_);
	loadIfDictionaryDefined(dict,"init_pose", &p->initialPose_);
}

void LuaLoader::loadParameters(const DictPtr dict, ScanToMapRegistrationParameters *p){
	std::string regTypeName = "";
	loadStringIfKeyDefined(dict, "scan_to_map_refinement_type", &regTypeName);
	p->scanToMapRegType_ = ScanToMapRegistrationStringToEnumMap.at(regTypeName);
	loadDoubleIfKeyDefined(dict, "min_refinement_fitness", &p->minRefinementFitness_);
	loadIfDictionaryDefined(dict,"icp", &p->icp_);
}


void LuaLoader::loadParameters(const DictPtr dict, MapBuilderParameters* p) {
	loadDoubleIfKeyDefined(dict, "map_voxel_size", &p->mapVoxelSize_);
	loadIfDictionaryDefined(dict,"space_carving", &p->carving_);
	loadIfDictionaryDefined(dict,"scan_cropping", &p->cropper_);
}

void LuaLoader::loadParameters(const DictPtr dict, SubmapParameters *p){
	loadDoubleIfKeyDefined(dict, "submap_size", &p->radius_);
	loadDoubleIfKeyDefined(dict, "adjacency_based_revisiting_min_fitness", &p->adjacencyBasedRevisitingMinFitness_);
	loadIntIfKeyDefined(dict, "submaps_num_scan_overlap", &p->numScansOverlap_);
	loadIntIfKeyDefined(dict, "min_num_range_data", &p->minNumRangeData_);

}

void LuaLoader::loadParameters(const DictPtr dict, OdometryParameters *p){
	loadIfDictionaryDefined(dict,"scan_matching", &p->scanMatcher_);
	loadIfDictionaryDefined(dict,"scan_processing", &p->scanProcessing_);
	loadBoolIfKeyDefined(dict, "is_publish_odometry_msgs", &p->isPublishOdometryMsgs_);
}

void LuaLoader::loadParameters(const DictPtr dict, CloudRegistrationParameters *p){
	std::string regTypeName ="";
	loadStringIfKeyDefined(dict, "cloud_registration_type", &regTypeName);
	p->regType_ = CloudRegistrationStringToEnumMap.at(regTypeName);
	loadIfDictionaryDefined(dict,"icp", &p->icp_);
}

void LuaLoader::loadParameters(const DictPtr dict, ScanProcessingParameters *p){
	loadDoubleIfKeyDefined(dict, "voxel_size", &p->voxelSize_);
	loadDoubleIfKeyDefined(dict, "downsampling_ratio", &p->downSamplingRatio_);
	loadIfDictionaryDefined(dict,"scan_cropping", &p->cropper_);
}

void LuaLoader::loadParameters(const DictPtr dict, SavingParameters *p){
	loadBoolIfKeyDefined(dict, "save_at_mission_end", &p->isSaveAtMissionEnd_);
	loadBoolIfKeyDefined(dict, "save_map", &p->isSaveMap_);
	loadBoolIfKeyDefined(dict, "save_submaps", &p->isSaveSubmaps_);
	loadBoolIfKeyDefined(dict, "save_dense_submaps", &p->isSaveDenseSubmaps_);
}

void LuaLoader::loadParameters(const DictPtr dict, VisualizationParameters *p){
	loadDoubleIfKeyDefined(dict, "assembled_map_voxel_size", &p->assembledMapVoxelSize_);
	loadDoubleIfKeyDefined(dict, "submaps_voxel_size", &p->submapVoxelSize_);
	loadDoubleIfKeyDefined(dict, "visualize_every_n_msec", &p->visualizeEveryNmsec_);

}

void LuaLoader::loadParameters(const DictPtr dict, ConstantVelocityMotionCompensationParameters *p){
	loadBoolIfKeyDefined(dict, "is_undistort_scan", &p->isUndistortInputCloud_);
	loadBoolIfKeyDefined(dict, "is_spinning_clockwise", &p->isSpinningClockwise_);
	loadDoubleIfKeyDefined(dict, "scan_duration", &p->scanDuration_);
	loadIntIfKeyDefined(dict, "num_poses_vel_estimation", &p->numPosesVelocityEstimation_);
}

void LuaLoader::loadParameters(const DictPtr dict, IcpParameters *p){
	loadIntIfKeyDefined(dict, "max_n_iter", &p->maxNumIter_);
	loadIntIfKeyDefined(dict, "knn", &p->knn_);
	loadDoubleIfKeyDefined(dict, "max_correspondence_dist", &p->maxCorrespondenceDistance_);
	loadDoubleIfKeyDefined(dict, "max_distance_knn", &p->maxDistanceKnn_);
}
void LuaLoader::loadParameters(const DictPtr dict, GlobalOptimizationParameters *p){
	loadDoubleIfKeyDefined(dict, "edge_prune_threshold", &p->edgePruneThreshold_);
	loadDoubleIfKeyDefined(dict, "max_correspondence_distance", &p->maxCorrespondenceDistance_);
	loadIntIfKeyDefined(dict, "reference_node", &p->referenceNode_);
	loadDoubleIfKeyDefined(dict, "loop_closure_preference", &p->loopClosurePreference_);
}

void LuaLoader::loadParameters(const DictPtr dict, PlaceRecognitionConsistencyCheckParameters *p){
	loadDoubleIfKeyDefined(dict, "max_drift_pitch", &p->maxDriftPitch_);
	loadDoubleIfKeyDefined(dict, "max_drift_roll", &p->maxDriftRoll_);
	loadDoubleIfKeyDefined(dict, "max_drift_yaw", &p->maxDriftYaw_);
	loadDoubleIfKeyDefined(dict, "max_drift_x", &p->maxDriftX_);
	loadDoubleIfKeyDefined(dict, "max_drift_y", &p->maxDriftY_);
	loadDoubleIfKeyDefined(dict, "max_drift_z", &p->maxDriftZ_);
	p->maxDriftRoll_ *= kDegToRad;
	p->maxDriftPitch_ *= kDegToRad;
	p->maxDriftYaw_ *= kDegToRad;
}

void LuaLoader::loadParameters(const DictPtr dict, ScanCroppingParameters *p){
	loadDoubleIfKeyDefined(dict, "cropping_radius_max", &p->croppingMaxRadius_);
	loadDoubleIfKeyDefined(dict, "cropping_radius_min", &p->croppingMinRadius_);
	loadDoubleIfKeyDefined(dict, "min_z", &p->croppingMinZ_);
	loadDoubleIfKeyDefined(dict, "max_z", &p->croppingMaxZ_);
	loadStringIfKeyDefined(dict, "cropper_type", &p->cropperName_);
}

void LuaLoader::loadParameters(const DictPtr dict, SpaceCarvingParameters *p){
	loadDoubleIfKeyDefined(dict, "voxel_size", &p->voxelSize_);
	loadDoubleIfKeyDefined(dict, "max_raytracing_length", &p->maxRaytracingLength_);
	loadDoubleIfKeyDefined(dict, "truncation_distance", &p->truncationDistance_);
	loadIntIfKeyDefined(dict, "carve_space_every_n_scans", &p->carveSpaceEveryNscans_);
}

void LuaLoader::loadParameters(const DictPtr dict, Eigen::Isometry3d *T) {
	Eigen::Isometry3d retVal = Eigen::Isometry3d::Identity();
	double roll(0.0), pitch(0.0), yaw(0.0);
	loadDoubleIfKeyDefined(dict, "roll", &roll);
	loadDoubleIfKeyDefined(dict, "pitch", &pitch);
	loadDoubleIfKeyDefined(dict, "yaw", &yaw);
	const Eigen::Quaterniond q = fromRPY(roll, pitch, yaw).normalized();
	retVal = makeTransform(Eigen::Vector3d::Zero(), q);

	double x(0.0), y(0.0), z(0.0);
	loadDoubleIfKeyDefined(dict, "x", &x);
	loadDoubleIfKeyDefined(dict, "y", &y);
	loadDoubleIfKeyDefined(dict, "z", &z);
	retVal.translation() = Eigen::Vector3d(x, y, z);

	*T = retVal;
}


} // namespace io_lua
} // nam

