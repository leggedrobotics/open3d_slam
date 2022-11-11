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


namespace o3d_slam {
namespace io_lua {

namespace {


const double kDegToRad = M_PI / 180.0;

} //namespace




void loadParameters(const std::string &folderpath, const std::string &topLevelFileName, SlamParameters *p){
	LuaLoader loader;
	loader.setupDictionary(topLevelFileName, folderpath);
	DictSharedPtr dict = loader.getDict();
	if (dict->HasKey("saving")){
		DictPtr subDict = dict->GetDictionary("saving");
		loadParameters(std::move(subDict), &p->saving_);
	} else {
		std::cout << "saving parameters not defined \n";
	}

	if (dict->HasKey("visualization")){
		DictPtr subDict = dict->GetDictionary("visualization");
		loadParameters(std::move(subDict), &p->visualization_);
	} else {
		std::cout << "visualization parameters not defined \n";
	}

	if (dict->HasKey("motion_compensation")){
		DictPtr subDict = dict->GetDictionary("motion_compensation");
		loadParameters(std::move(subDict), &p->motionCompensation_);
	} else {
		std::cout << "motion_compensation parameters not defined \n";
	}

	if (dict->HasKey("global_optimization")){
		DictPtr subDict = dict->GetDictionary("global_optimization");
		loadParameters(std::move(subDict), &p->mapper_.globalOptimization_);
	} else {
		std::cout << "motion_compensation parameters not defined \n";
	}
}

void loadParameters(const DictPtr dict, SavingParameters *p){
	p->isSaveAtMissionEnd_ = dict->GetBool("save_at_mission_end");
	p->isSaveMap_ = dict->GetBool("save_map");
	p->isSaveSubmaps_ = dict->GetBool("save_submaps");
}

void loadParameters(const DictPtr dict, VisualizationParameters *p){
	p->assembledMapVoxelSize_ = dict->GetDouble("assembled_map_voxel_size");
	p->submapVoxelSize_ = dict->GetDouble("submaps_voxel_size");
	p->visualizeEveryNmsec_ = dict->GetDouble("visualize_every_n_msec");
}

void loadParameters(const DictPtr dict, ConstantVelocityMotionCompensationParameters *p){
	p->isUndistortInputCloud_ = dict->GetBool("is_undistort_scan");
	p->isSpinningClockwise_ = dict->GetBool("is_spinning_clockwise");
	p->scanDuration_ = dict->GetDouble("scan_duration");
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


void loadParameters(const std::string &filename, MapperParameters *p){
//	YAML::Node basenode = YAML::LoadFile(filename);
//	if (basenode.IsNull()) {
//		throw std::runtime_error("MapperParameters::loadParameters loading failed");
//	}
//	if (!basenode["mapping"].IsDefined()){
//		std::cout << "mapping not defined \n";
//		return;
//	}
//	loadParameters(basenode["mapping"], p);
}



} // namespace io_lua
} // namespace o3d_slam
