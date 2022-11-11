/*
 * helpers.hpp
 *
 *  Created on: Nov 11, 2022
 *      Author: jelavice
 */

#pragma once
#include "open3d_slam_lua_io/LuaLoader.hpp"
#include <iostream>

namespace o3d_slam {
namespace io_lua{

template<typename Ret>
void loadIfKeyDefined(const DictPtr &dict, const std::string &key, Ret *value) {
	if (dict->HasKey(key)) {
			throw std::runtime_error("unknown type!!! only int, double and std::string are supported");
	} else {
		std::cout << " key " << key << " not found \n";;
	}
}

template<>
void loadIfKeyDefined<int>(const DictPtr &dict, const std::string &key, int *value) {
	if (dict->HasKey(key)) {
			*value = dict->GetInt(key);
	} else {
		std::cout << " key " << key << " not found \n";;
	}
}

template<>
void loadIfKeyDefined<double>(const DictPtr &dict, const std::string &key, double *value) {
	if (dict->HasKey(key)) {
			*value = dict->GetDouble(key);
	} else {
		std::cout << " key " << key << " not found \n";;
	}
}

template<>
void loadIfKeyDefined<std::string>(const DictPtr &dict, const std::string &key, std::string *value) {
	if (dict->HasKey(key)) {
			*value = dict->GetString(key);
	} else {
		std::cout << " key " << key << " not found \n";;
	}
}

} // namespace io_lua
} // namespace o3d_slam
