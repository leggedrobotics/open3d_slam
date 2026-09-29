/*
 * TransformInterpolationBuffer.cpp
 *
 *  Created on: Nov 9, 2021
 *      Author: jelavice
 */
#include "open3d_slam/TransformInterpolationBuffer.hpp"
#include "open3d_slam/assert.hpp"
#include "open3d_slam/time.hpp"

#include <algorithm>
#include <iostream>
#include <stdexcept>

namespace o3d_slam {

TransformInterpolationBuffer::TransformInterpolationBuffer() : TransformInterpolationBuffer(2000) {}

TransformInterpolationBuffer::TransformInterpolationBuffer(size_t bufferSize) {
  setSizeLimit(bufferSize);
}

void TransformInterpolationBuffer::push(const Time& time, const Transform& tf) {
  std::lock_guard<std::mutex> lock(modifierMutex_);
  // this relies that they will be pushed in order!!!
  if (!transforms_.empty()) {
    if (time < transforms_.front().time_) {
      std::cerr << "TransformInterpolationBuffer:: you are trying to push something earlier than the earliest measurement, this should not "
                   "happen \n";
      std::cerr << "ingnoring the mesurement \n";
      std::cerr << "Time: " << toSecondsSinceFirstMeasurement(time) << std::endl;
      std::cerr << "earliest time: " << toSecondsSinceFirstMeasurement(transforms_.front().time_) << std::endl;
      return;
    }

    if (time < transforms_.back().time_) {
      std::cerr << "TransformInterpolationBuffer:: you are trying to push something out of order, this should not happen \n";
      std::cerr << "ingnoring the mesurement \n";
      std::cerr << "Time: " << time << std::endl;
      std::cerr << "latest time: " << toSecondsSinceFirstMeasurement(transforms_.back().time_) << std::endl;
      return;
    }
  }
  transforms_.push_back({time, tf});
  removeOldMeasurementsIfNeeded();
}

void TransformInterpolationBuffer::applyToAllElementsInTimeInterval(const Transform& t, const Time& begin, const Time& end) {
  std::lock_guard<std::mutex> lock(modifierMutex_);
  //	assert_ge(toUniversal(end),toUniversal(begin));
  for (auto it = transforms_.begin(); it != transforms_.end(); ++it) {
    if (it->time_ >= begin && it->time_ <= end) {
      it->transform_ = it->transform_ * t;
    }
  }
}

void TransformInterpolationBuffer::setSizeLimit(const size_t buffer_size_limit) {
  std::lock_guard<std::mutex> lock(modifierMutex_);
  bufferSizeLimit_ = buffer_size_limit;
  removeOldMeasurementsIfNeeded();
}

void TransformInterpolationBuffer::clear() {
  std::lock_guard<std::mutex> lock(modifierMutex_);
  transforms_.clear();
}

TimestampedTransform TransformInterpolationBuffer::latest_measurement(int offsetFromLastElement /*=0*/) const {
  std::lock_guard<std::mutex> lock(modifierMutex_);
  if (transforms_.empty()) {
    throw std::runtime_error("TransformInterpolationBuffer:: latest_measurement: Empty buffer");
  }
  if (offsetFromLastElement < 0 || static_cast<size_t>(offsetFromLastElement) >= transforms_.size()) {
    throw std::out_of_range("TransformInterpolationBuffer:: latest_measurement: offset out of range");
  }
  return *(std::prev(transforms_.end(), offsetFromLastElement + 1));
}

bool TransformInterpolationBuffer::has(const Time& time) const {
  std::lock_guard<std::mutex> lock(modifierMutex_);
  if (transforms_.empty()) {
    return false;
  }
  return transforms_.front().time_ <= time && time <= transforms_.back().time_;
}

Transform TransformInterpolationBuffer::lookup(const Time& time) const {
  std::lock_guard<std::mutex> lock(modifierMutex_);
  if (transforms_.empty() || time < transforms_.front().time_ || time > transforms_.back().time_) {
    throw std::runtime_error("TransformInterpolationBuffer:: Missing transform for: " + toString(time));
  }

  if (transforms_.size() == 1) {
    return transforms_.front().transform_;
  }

  const auto getMeasurement =
      std::lower_bound(transforms_.begin(), transforms_.end(), time,
                       [](const TimestampedTransform& tf, const Time& requestedTime) { return tf.time_ < requestedTime; });
  if (getMeasurement != transforms_.end() && getMeasurement->time_ == time) {
    return getMeasurement->transform_;
  }
  const auto start = std::prev(getMeasurement);
  return interpolate(*start, *getMeasurement, time).transform_;
}

Transform TransformInterpolationBuffer::lookupOrClamp(const Time& time) const {
  std::lock_guard<std::mutex> lock(modifierMutex_);
  if (transforms_.empty()) {
    throw std::runtime_error("TransformInterpolationBuffer:: Missing transform for: " + toString(time));
  }
  if (time <= transforms_.front().time_) {
    return transforms_.front().transform_;
  }
  if (time >= transforms_.back().time_) {
    return transforms_.back().transform_;
  }

  const auto end = std::lower_bound(transforms_.begin(), transforms_.end(), time,
                                    [](const TimestampedTransform& tf, const Time& requestedTime) { return tf.time_ < requestedTime; });
  if (end->time_ == time) {
    return end->transform_;
  }
  return interpolate(*std::prev(end), *end, time).transform_;
}

void TransformInterpolationBuffer::removeOldMeasurementsIfNeeded() {
  while (transforms_.size() > bufferSizeLimit_) {
    transforms_.pop_front();
  }
}

Time TransformInterpolationBuffer::earliest_time() const {
  std::lock_guard<std::mutex> lock(modifierMutex_);
  if (transforms_.empty()) {
    throw std::runtime_error("TransformInterpolationBuffer:: Empty buffer");
  }
  return transforms_.front().time_;
}

Time TransformInterpolationBuffer::latest_time() const {
  std::lock_guard<std::mutex> lock(modifierMutex_);
  if (transforms_.empty()) {
    throw std::runtime_error("TransformInterpolationBuffer:: Empty buffer");
  }
  return transforms_.back().time_;
}

bool TransformInterpolationBuffer::empty() const {
  std::lock_guard<std::mutex> lock(modifierMutex_);
  return transforms_.empty();
}

size_t TransformInterpolationBuffer::size_limit() const {
  std::lock_guard<std::mutex> lock(modifierMutex_);
  return bufferSizeLimit_;
}

size_t TransformInterpolationBuffer::size() const {
  std::lock_guard<std::mutex> lock(modifierMutex_);
  return transforms_.size();
}

void TransformInterpolationBuffer::printTimesCurrentlyInBuffer() const {
  std::lock_guard<std::mutex> lock(modifierMutex_);
  for (auto it = transforms_.cbegin(); it != transforms_.cend(); ++it) {
    std::cout << toSecondsSinceFirstMeasurement(it->time_) << std::endl;
  }
}

Transform getTransform(const Time& time, const TransformInterpolationBuffer& buffer) {
  return buffer.lookupOrClamp(time);
}

}  // namespace o3d_slam
