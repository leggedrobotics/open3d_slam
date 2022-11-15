/*
 * LuaLoader.cpp
 *
 *  Created on: Nov 11, 2022
 *      Author: jelavice
 */

#include "open3d_slam_lua_io/LuaLoader.hpp"
#include "open3d_slam_lua_io/helpers.hpp"
#include <iostream>
#include <stack>

namespace o3d_slam {
namespace io_lua {

namespace {
std::stack<std::string> S;

} // namespace

using namespace lua_dict;

void LuaLoader::setupDictionary(const std::string &topLevelFileName, const std::string &folderPath) {
	const std::vector<std::string> paths( { folderPath });
	auto fileResolver = std::make_unique<ConfigurationFileResolver>(paths);
	const std::string fullContent = fileResolver->GetFileContentOrDie(topLevelFileName);
	const std::string fullPath = fileResolver->GetFullPathOrDie(topLevelFileName);
//	dict_ = std::make_shared<LuaParameterDictionary>(fullContent, std::move(fileResolver));
	dict_ = LuaParameterDictionary::NonReferenceCounted(fullContent, std::move(fileResolver));

	std::cout << "Lua loader resolved full path, loading from: " << fullPath << std::endl;
	 topFileName_ = topLevelFileName;
	folderPath_ = folderPath;
}

const DictPtr& LuaLoader::getDict() const {
	return dict_;
}
void LuaLoader::buildLuaParamList(){
	if (S.empty()){
		S.push("o3d_params");
	}
	const std::vector<std::string> paths( { folderPath_ });
	auto fileResolver = std::make_unique<ConfigurationFileResolver>(paths);
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
			S.push(key);
			treeTraversal(subDict);
		} else {
			auto name = extractPrefix(S) + "/" + key;
			luaParamList_.insert(name);
//			std::cout << "Name: " << name << std::endl;
		}
	}
	S.pop();
}

} // namespace io_lua
} // nam

