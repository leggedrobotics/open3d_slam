/*
 * CircularBuffer.hpp
 *
 *  Created on: Nov 23, 2021
 *      Author: jelavice
 */

#pragma once
#include <deque>
#include <mutex>
#include <stdexcept>
#include <utility>

namespace o3d_slam {

template <typename T>
class CircularBuffer {
 public:
  CircularBuffer() = default;
  void set_size_limit(size_t size) {
    std::lock_guard<std::mutex> lck(mutex_);
    bufferSizeLimit_ = size;
    removeOldMeasurementsIfNeeded();
  }

  void push(const T& data) {
    std::lock_guard<std::mutex> lck(mutex_);
    data_.push_back(data);
    removeOldMeasurementsIfNeeded();
  }

  void push(T&& data) {
    std::lock_guard<std::mutex> lck(mutex_);
    data_.push_back(std::move(data));
    removeOldMeasurementsIfNeeded();
  }

  T peek_front() const {
    std::lock_guard<std::mutex> lck(mutex_);
    if (data_.empty()) {
      throw std::runtime_error("CircularBuffer::peek_front: empty buffer");
    }
    return data_.front();
  }

  T peek_back() const {
    std::lock_guard<std::mutex> lck(mutex_);
    if (data_.empty()) {
      throw std::runtime_error("CircularBuffer::peek_back: empty buffer");
    }
    return data_.back();
  }

  T pop() {
    std::lock_guard<std::mutex> lck(mutex_);
    if (data_.empty()) {
      throw std::runtime_error("CircularBuffer::pop: empty buffer");
    }
    T copy = std::move(data_.front());
    data_.pop_front();
    return copy;
  }

  bool empty() const {
    std::lock_guard<std::mutex> lck(mutex_);
    return data_.empty();
  }

  size_t size_limit() const {
    std::lock_guard<std::mutex> lck(mutex_);
    return bufferSizeLimit_;
  }

  size_t size() const {
    std::lock_guard<std::mutex> lck(mutex_);
    return data_.size();
  }

  void clear() {
    std::lock_guard<std::mutex> lck(mutex_);
    data_.clear();
  }

  std::deque<T> getImplementation() const {
    std::lock_guard<std::mutex> lck(mutex_);
    return data_;
  }

 private:
  void removeOldMeasurementsIfNeeded() {
    while (data_.size() > bufferSizeLimit_) {
      data_.pop_front();
    }
  }

  std::deque<T> data_;
  mutable std::mutex mutex_;
  size_t bufferSizeLimit_ = 10;
};

}  // namespace o3d_slam
