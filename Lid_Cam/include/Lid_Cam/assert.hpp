/*
 * assert.hpp
 *
 *  Created on: Nov 9, 2021
 *      Author: jelavice
 */

#pragma once
#include <stdexcept>
#include <string>
namespace o3d_slam {

template<typename T>
inline void assert_ge(T val, T ref, const std::string &errMsg = "") {
	if (val < ref) {
		const std::string out = errMsg + " val: " + std::to_string(val) + " is not ge than: " + std::to_string(ref);
		throw std::runtime_error(out);
	}
}

template<typename T>
inline void assert_gt(T val, T ref, const std::string &errMsg = "") {
	if (val <= ref) {
		const std::string out = errMsg + " val: " + std::to_string(val) + " is not gt than: " + std::to_string(ref);
		throw std::runtime_error(out);
	}
}

template<typename T>
inline void assert_le(T val, T ref, const std::string &errMsg = "") {
	if (val > ref) {
		const std::string out = errMsg + " val: " + std::to_string(val) + " is not le than: " + std::to_string(ref);
		throw std::runtime_error(out);
	}
}

template<typename T>
inline void assert_lt(T val, T ref, const std::string &errMsg = "") {
	if (val >= ref) {
		const std::string out = errMsg + " val: " + std::to_string(val) + " is not lt than: " + std::to_string(ref);
		throw std::runtime_error(out);
	}
}

template<typename T>
inline void assert_nonNullptr(T ptr, const std::string &errMsg = "") {
	if (ptr == nullptr) {
		throw std::runtime_error(errMsg);
	}
}

inline void assert_true(bool var, const std::string &errMsg = "") {
	if (!var) {
		throw std::runtime_error(errMsg);
	}
}

template<typename T>
inline void assert_eq(T val, T ref, const std::string &prefix = "") {
	if (val != ref) {
		const std::string errMsg = prefix + ", val: " + std::to_string(val) + " is not eq to: " + std::to_string(ref);
		throw std::runtime_error(errMsg);
	}
}

} // namespace o3d_slam
