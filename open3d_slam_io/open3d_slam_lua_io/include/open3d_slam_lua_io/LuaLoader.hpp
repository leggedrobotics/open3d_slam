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
private:
	DictPtr dict_;

};

} // namespace io_lua
} // namespace o3d_slam
