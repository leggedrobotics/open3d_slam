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

std::string getKeysAsStringCsv(const lua_dict::LuaParameterDictionary &dict){
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

template<typename Ret>
void loadIfKeyDefined(const DictPtr &dict, const std::string &key, Ret *value) {
	if (dict->HasKey(key)) {
			throw std::runtime_error("unknown type!!! only int, bool, double and std::string are supported");
	} else {
		std::cout << " [WARNING] PARAM LOAD SINGLE KEY: key " << key << " not found \n";;
		const std::string keysAvailable = getKeysAsStringCsv(*dict);
		std::cout << "	keys availalbe: " << keysAvailable << "\n\n";
	}
}

template<>
void loadIfKeyDefined<int>(const DictPtr &dict, const std::string &key, int *value) {
	if (dict->HasKey(key)) {
			*value = dict->GetInt(key);
	} else {
		std::cout << " [WARNING] PARAM LOAD SINGLE KEY: key " << key << " not found \n";;
		const std::string keysAvailable = getKeysAsStringCsv(*dict);
		std::cout << "	keys availalbe: " << keysAvailable << "\n\n";
	}
}

template<>
void loadIfKeyDefined<double>(const DictPtr &dict, const std::string &key, double *value) {
	if (dict->HasKey(key)) {
			*value = dict->GetDouble(key);
	} else {
		std::cout << " [WARNING] PARAM LOAD SINGLE KEY: key " << key << " not found \n";;
		const std::string keysAvailable = getKeysAsStringCsv(*dict);
		std::cout << "	keys availalbe: " << keysAvailable << "\n\n";
	}
}

template<>
void loadIfKeyDefined<std::string>(const DictPtr &dict, const std::string &key, std::string *value) {
	if (dict->HasKey(key)) {
			*value = dict->GetString(key);
	} else {
		std::cout << " [WARNING] PARAM LOAD SINGLE KEY: key " << key << " not found \n";;
		const std::string keysAvailable = getKeysAsStringCsv(*dict);
		std::cout << "	keys availalbe: " << keysAvailable << "\n\n";
	}
}

template<>
void loadIfKeyDefined<bool>(const DictPtr &dict, const std::string &key, bool *value) {
	if (dict->HasKey(key)) {
			*value = dict->GetBool(key);
	} else {
		std::cout << " [WARNING] PARAM LOAD SINGLE KEY: key " << key << " not found \n";;
		const std::string keysAvailable = getKeysAsStringCsv(*dict);
		std::cout << "	keys availalbe: " << keysAvailable << "\n\n";
	}
}

template<typename Param>
void loadIfDictionaryDefined(const DictPtr &dict,const std::string &key, Param *p){
	if (dict->HasKey(key)){
		DictPtr subDict = dict->GetDictionary(key);
		loadParameters(std::move(subDict), p);
	} else {
		std::cout << "[WARNING] PARAM LOAD DICT: " << key << " sub-dictionary not defined \n";
		const std::string keysAvailable = getKeysAsStringCsv(*dict);
		std::cout << "	keys availalbe: " << keysAvailable << "\n\n";
	}
}

template<typename Param>
void loadIfDictionaryDefinedMultiLevel(const DictPtr &dict, const std::vector<std::string> &keys, Param *p) {
	DictPtr subDict;
	for (size_t i = 0; i < keys.size(); ++i) {
		const auto &key = keys.at(i);
		const bool hasKey = i == 0 ? dict->HasKey(key) : subDict->HasKey(key);
		if (hasKey) {
			subDict = i == 0 ? dict->GetDictionary(key) : subDict->GetDictionary(key);
			if (i == keys.size() - 1) {
				loadParameters(std::move(subDict), p);
			}
		} else {
			std::cout << "[WARNING] PARAM LOAD MULTI-LVL-DICT: " << key << " sub-dictionary not defined \n";
		}
	}
}

bool isKeyDefinedMultiLevel(const DictPtr &dict, const std::vector<std::string> &keys) {
	DictPtr subDict;
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
