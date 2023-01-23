/*
 * helpers.hpp
 *
 *  Created on: Nov 11, 2022
 *      Author: jelavice
 */

#pragma once
#include "lua_parameter_dictionary/lua_parameter_dictionary.h"
#include <iostream>
#include <stack>

namespace o3d_slam {
namespace io_lua{

inline std::string getKeysAsStringCsv(const lua_dict::LuaParameterDictionary &dict){
	const std::vector<std::string> keys = dict.GetKeys();
	std::string retVal = "";
	for (size_t i =0; i < keys.size(); ++i){
		retVal += keys.at(i);
		if (i < keys.size()-1){
			retVal += ", ";
		}
	}
	return retVal;
}

inline bool isKeyDefinedMultiLevel(const std::unique_ptr<lua_dict::LuaParameterDictionary> &dict, const std::vector<std::string> &keys) {
	std::unique_ptr<lua_dict::LuaParameterDictionary> subDict;
	if (keys.size() == 1) {
		return dict->HasKey(keys.at(0));
	}
	for (size_t i = 0; i < keys.size(); ++i) {
		const auto &key = keys.at(i);
		const bool isFirst = i == 0;
		const bool isLast = i == keys.size() - 1;
		const bool hasKey = isFirst ? dict->HasKey(key) : subDict->HasKey(key);
		if (isLast){
			return subDict->HasKey(key);
		}
		subDict = isFirst ? dict->GetDictionary(key) : subDict->GetDictionary(key);
	}
	return false;
}


} // namespace io_lua
} // namespace o3d_slam
