/*
 * LuaLoader.cpp
 *
 *  Created on: Nov 11, 2022
 *      Author: jelavice
 */

#include "open3d_slam_lua_io/LuaLoader.hpp"
#include <iostream>


namespace o3d_slam {
namespace io_lua{

using namespace lua_dict;

void LuaLoader::setupDictionary(const std::string &topLevelFileName, const std::string &folderPath) {
	const std::vector<std::string> paths({folderPath});
	auto fileResolver = std::make_unique<ConfigurationFileResolver>(paths);
	const std::string fullContent = fileResolver->GetFileContentOrDie(topLevelFileName);
	const std::string fullPath = fileResolver->GetFullPathOrDie(topLevelFileName);
//	dict_ = std::make_shared<LuaParameterDictionary>(fullContent, std::move(fileResolver));
	dict_ = LuaParameterDictionary::NonReferenceCounted(fullContent, std::move(fileResolver));

  std::cout << "Lua loader resolved full path, loading from: " << fullPath << std::endl;
}

const DictPtr &LuaLoader::getDict() const{
	return dict_;
}

} // namespace io_lua
} // nam


