/*
 * ThreadSafeBuffer.hpp
 *
 *  Created on: Nov 16, 2021
 *      Author: jelavice
 */

#pragma once
#include <iostream>
#include <mutex>
#include <vector>

namespace o3d_slam {

template <typename T>
class ThreadSafeBuffer {
 public:
  void push(const T& val) {
    std::lock_guard<std::mutex> lck(modifierMutex_);
    data_.push_back(val);
  }

  template <typename InputIt>
  void insert(InputIt first, InputIt last) {
    std::lock_guard<std::mutex> lck(modifierMutex_);
    data_.insert(data_.end(), first, last);
  }

  std::vector<T> peek() const {
    std::lock_guard<std::mutex> lck(modifierMutex_);
    return data_;
  }

  void clear() {
    std::lock_guard<std::mutex> lck(modifierMutex_);
    data_.clear();
  }

  std::vector<T> popAllElements() {
    std::lock_guard<std::mutex> lck(modifierMutex_);
    std::vector<T> elements;
    elements.swap(data_);
    return elements;
  }

  bool empty() const {
    std::lock_guard<std::mutex> lck(modifierMutex_);
    return data_.empty();
  }

  size_t size() const {
    std::lock_guard<std::mutex> lck(modifierMutex_);
    return data_.size();
  }

 private:
  std::vector<T> data_;
  mutable std::mutex modifierMutex_;
};

}  // namespace o3d_slam
