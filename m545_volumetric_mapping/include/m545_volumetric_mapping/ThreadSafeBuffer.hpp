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

namespace m545_mapping {



template<typename T>
class ThreadSafeBuffer {

public:
	void push(const T &val) {
		std::lock_guard<std::mutex> lck(modifierMutex_);
		buffer_.push_back(val);
	}

	const std::vector<T> &peek () const {
		return buffer_;
	}

	void clear() {
		std::lock_guard<std::mutex> lck(modifierMutex_);
		buffer_.clear();
	}

	const std::vector<T> popAllElements(){
		std::lock_guard<std::mutex> lck(modifierMutex_);
		auto copy = buffer_;
		buffer_.clear();
		return copy;
	}

	bool empty() const {
		return buffer_.empty();
	}


private:
	std::vector<T> buffer_;
	std::mutex modifierMutex_;
};


} //namespace m545_mapping


