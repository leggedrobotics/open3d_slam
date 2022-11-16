/*
 * LuaLoader.hpp
 *
 *  Created on: Nov 11, 2022
 *      Author: jelavice
 */

#pragma once
#include <string>
#include "lua_parameter_dictionary/configuration_file_resolver.h"
#include "lua_parameter_dictionary/lua_parameter_dictionary.h"
#include "open3d_slam_lua_io/helpers.hpp"
#include "open3d_slam/Parameters.hpp"

#include <unordered_map>
#include <set>
#include <stack>

namespace o3d_slam {
namespace io_lua{

using DictPtr = std::unique_ptr<lua_dict::LuaParameterDictionary>;
using DictSharedPtr = std::shared_ptr<lua_dict::LuaParameterDictionary>;

class LuaLoader{

public:
	LuaLoader() = default;
	~LuaLoader() = default;
	void setupDictionary(const std::string &topLevelFileName, const std::string &folderPath);
	const DictPtr &getDict() const;
	void buildLuaParamList();
	void loadParameters(const std::string &folderpath, const std::string &topLevelFileName, SlamParameters *p);
	bool isLoadingOkay();
private:
	void treeTraversal(const DictPtr &dict);
	void loadIntIfKeyDefined(const DictPtr &dict, const std::string &key, int *value);
	void loadDoubleIfKeyDefined(const DictPtr &dict, const std::string &key, double *value);
	void loadStringIfKeyDefined(const DictPtr &dict, const std::string &key, std::string *value);
	void loadBoolIfKeyDefined(const DictPtr &dict, const std::string &key, bool *value);
	void incrementRefCount(const std::string &key);


	void loadParameters(const DictPtr dict, ConstantVelocityMotionCompensationParameters *p);
	void loadParameters(const DictPtr dict, SavingParameters *p);
	void loadParameters(const DictPtr dict, PlaceRecognitionConsistencyCheckParameters *p);
	void loadParameters(const DictPtr dict, PlaceRecognitionParameters *p);
	void loadParameters(const DictPtr dict, GlobalOptimizationParameters *p);
	void loadParameters(const DictPtr dict, VisualizationParameters *p);
	void loadParameters(const DictPtr dict, SubmapParameters *p);
	void loadParameters(const DictPtr dict, ScanProcessingParameters *p);
	void loadParameters(const DictPtr dict, IcpParameters *p);
	void loadParameters(const DictPtr dict, CloudRegistrationParameters *p);
	void loadParameters(const DictPtr dict, MapperParameters *p);
	void loadParameters(const DictPtr dict, MapBuilderParameters *p);
	void loadParameters(const DictPtr dict, OdometryParameters *p);
	void loadParameters(const DictPtr dict, SpaceCarvingParameters *p);
	void loadParameters(const DictPtr dict, ScanCroppingParameters *p);
	void loadParameters(const DictPtr dict, ScanToMapRegistrationParameters *p);
	void loadParameters(const DictPtr dict, MapInitializingParameters *p);
	void loadParameters(const DictPtr dict, Eigen::Isometry3d* T);


	template<typename Param>
	void loadIfDictionaryDefined(const DictPtr &dict,const std::string &key, Param *p);
	template<typename Param>
	void loadIfDictionaryDefinedMultiLevel(const DictPtr &dict, const std::vector<std::string> &keys, Param *p);

	DictPtr dict_;
	std::string topFileName_;
	std::vector<std::string> folderPaths_;
	std::map<std::string,int> loadingParamCount_;
	std::stack<std::string> loadingNameStack_;
	std::set<std::string> luaParamList_;
};



template<typename Param>
void LuaLoader::loadIfDictionaryDefined(const DictPtr &dict,const std::string &key, Param *p){
	if (dict->HasKey(key)){
		loadingNameStack_.push(key);
		DictPtr subDict = dict->GetDictionary(key);
		loadParameters(std::move(subDict), p);
	} else {
		std::cout << "\033[1;33m[WARNING]:\033[0m PARAM LOAD DICT: " << key << " sub-dictionary not defined \n";
		const std::string keysAvailable = o3d_slam::io_lua::getKeysAsStringCsv(*dict);
		std::cout << "	keys availalbe: " << keysAvailable << "\n\n";
	}
	loadingNameStack_.pop();
}

template<typename Param>
void LuaLoader::loadIfDictionaryDefinedMultiLevel(const DictPtr &dict, const std::vector<std::string> &keys, Param *p) {
	DictPtr subDict;
	int pushCount = 0;
	for (size_t i = 0; i < keys.size(); ++i) {
		const auto &key = keys.at(i);
		const bool hasKey = i == 0 ? dict->HasKey(key) : subDict->HasKey(key);
		if (hasKey) {
			loadingNameStack_.push(key);
			pushCount++;
			subDict = i == 0 ? dict->GetDictionary(key) : subDict->GetDictionary(key);
			if (i == keys.size() - 1) {
				loadParameters(std::move(subDict), p);
			}
		} else {
			std::cout << "\033[1;33m[WARNING]:\033[0m PARAM LOAD MULTI-LVL-DICT: " << key << " sub-dictionary not defined \n";
		}
	}
	for (int i =0; i < pushCount; ++i){
		loadingNameStack_.pop();
	}
}

} // namespace io_lua
} // namespace o3d_slam
