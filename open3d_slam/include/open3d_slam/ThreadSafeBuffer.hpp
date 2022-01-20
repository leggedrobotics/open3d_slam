/*
 * ThreadSafeBuffer.hpp
 *
 *  Created on: Nov 16, 2021
 *      Author: jelavice
 */

#pragma once
#include <iostream>
#include <vector>
#include <mutex>

namespace o3d_slam {



template<typename T>
class ThreadSafeBuffer {

public:
	void push(const T &val) {
		std::lock_guard<std::mutex> lck(modifierMutex_);
		data_.push_back(val);
	}

	template <typename InputIt>
	void insert(InputIt first, InputIt last) {
		std::lock_guard<std::mutex> lck(modifierMutex_);
		data_.insert(data_.end(),first, last);
	}

	const std::vector<T> &peek () const {
		return data_;
	}

	void clear() {
		std::lock_guard<std::mutex> lck(modifierMutex_);
		data_.clear();
	}

	const std::vector<T> popAllElements() {
		std::lock_guard<std::mutex> lck(modifierMutex_);
		auto copy = data_;
		data_.clear();
		return copy;
	}

	bool empty() const {
		return data_.empty();
	}

	size_t size() const {
		return data_.size();
	}


private:
	std::vector<T> data_;
	std::mutex modifierMutex_;
};


} //namespace o3d_slam


