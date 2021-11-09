/*
 * assert.hpp
 *
 *  Created on: Nov 9, 2021
 *      Author: jelavice
 */

#pragma once
#include <stdexcept>
#include <string>
namespace m545_mapping {

template<typename T>
void assert_ge(T val, T ref)
{
  if (val < ref) {
    const std::string errMsg = "val: " + std::to_string(val) + " is not ge than: "
        + std::to_string(ref);
    throw std::runtime_error(errMsg);
  }
}

template<typename T>
void assert_gt(T val, T ref)
{
  if (val <= ref) {
    const std::string errMsg = "val: " + std::to_string(val) + " is not gt than: "
        + std::to_string(ref);
    throw std::runtime_error(errMsg);
  }
}

template<typename T>
void assert_le(T val, T ref)
{
  if (val > ref) {
    const std::string errMsg = "val: " + std::to_string(val) + " is not le than: "
        + std::to_string(ref);
    throw std::runtime_error(errMsg);
  }
}

template<typename T>
void assert_lt(T val, T ref)
{
  if (val >= ref) {
    const std::string errMsg = "val: " + std::to_string(val) + " is not lt than: "
        + std::to_string(ref);
    throw std::runtime_error(errMsg);
  }
}
} // namespace m545_mapping
