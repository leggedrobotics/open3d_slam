/*
 * CircularBuffer.hpp
 *
 *  Created on: Nov 23, 2021
 *      Author: jelavice
 */

#pragma once
#include <deque>

namespace o3d_slam {

template<typename T>
class CircularBuffer {

public:
	CircularBuffer() = default;
	void set_size_limit(size_t size) {
		bufferSizeLimit_ = size;
		removeOldMeasurementsIfNeeded();
	}

	void push(const T &data) {
		{
			std::lock_guard<std::mutex> lck(pushMutex_);
			data_.push_back(data);
		}
		removeOldMeasurementsIfNeeded();
	}

	const T& peek_front() const {
		return data_.front();
	}

	const T& peek_back() const {
		return data_.back();
	}

	T pop() {
		std::lock_guard<std::mutex> lck(removeMutex_);
		T copy = data_.front();
		data_.pop_front();
		return copy;
	}

	bool empty() const {
		return data_.empty();
	}

	size_t size_limit() const {
		return bufferSizeLimit_;
	}

	size_t size() const {
		return data_.size();
	}

	void clear() {
		std::lock_guard<std::mutex> lck(removeMutex_);
		data_.clear();
	}

	const std::deque<T>& getImplementation() const {
		return data_;
	}

	std::deque<T>* getImplementationPtr() {
		return &data_;
	}

private:

	void removeOldMeasurementsIfNeeded() {
		std::lock_guard<std::mutex> lck(removeMutex_);
		while (data_.size() > bufferSizeLimit_) {
			data_.pop_front();
		}
	}

	std::deque<T> data_;
	std::mutex removeMutex_, pushMutex_;
	size_t bufferSizeLimit_ = 10;
};

} // namespace o3d_slam
