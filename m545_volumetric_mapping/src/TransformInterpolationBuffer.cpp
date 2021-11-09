/*
 * TransformInterpolationBuffer.cpp
 *
 *  Created on: Nov 9, 2021
 *      Author: jelavice
 */

#include "m545_volumetric_mapping/TransformInterpolationBuffer.hpp"
#include "m545_volumetric_mapping/time.hpp"

namespace m545_mapping {

TransformInterpolationBuffer::TransformInterpolationBuffer() :
		TransformInterpolationBuffer(2000) {
}


TransformInterpolationBuffer::TransformInterpolationBuffer(size_t bufferSize) {
	setSizeLimit(bufferSize);
}

void TransformInterpolationBuffer::push(const Time &time, const Transform &tf) {

	//this relies that they will be pushed in order!!!
	if (!transforms_.empty()) {
		if (time < earliest_time()) {
			std::cerr
					<< "TransformInterpolationBuffer:: you are trying to push something earlier than the earliest measurement, this should not happen \n";
			std::cerr << "ingnoring the mesurement \n";
			std::cerr << "Time: " << time << std::endl;
			std::cerr << "earliest time: " << earliest_time() << std::endl;
			return;
		}

		if (time < latest_time()) {
			std::cerr
					<< "TransformInterpolationBuffer:: you are trying to push something out of order, this should not happen \n";
			std::cerr << "ingnoring the mesurement \n";
			std::cerr << "Time: " << time << std::endl;
			std::cerr << "latest time: " << latest_time() << std::endl;
			return;
		}
	}
	transforms_.push_back( { time, tf });
	removeOldMeasurementsIfNeeded();
}

void TransformInterpolationBuffer::setSizeLimit(
		const size_t buffer_size_limit) {

	bufferSizeLimit_ = buffer_size_limit;
	removeOldMeasurementsIfNeeded();
}

void TransformInterpolationBuffer::clear() {
	transforms_.clear();
}

const TimestampedTransform& TransformInterpolationBuffer::latest_measurement(
		int n) const {
	if (empty()) {
		throw std::runtime_error("TransformInterpolationBuffer:: latest_measurement: Empty buffer");
	}
	return *(std::prev(transforms_.end(), n));
}
TimestampedTransform& TransformInterpolationBuffer::latest_measurement(int n) {
	if (empty()) {
		throw std::runtime_error("TransformInterpolationBuffer:: latest_measurement: Empty buffer");
	}
	return *(std::prev(transforms_.end(), n));
}

bool TransformInterpolationBuffer::has(const Time &time) const {
	if (transforms_.empty()) {
		return false;
	}
	return earliest_time() <= time && time <= latest_time();
}

Transform TransformInterpolationBuffer::lookup(const Time &time) const {

	if (!has(time)) {
		throw std::runtime_error(
				"TransformInterpolationBuffer:: Missing transform for: "
						+ toString(time));
	}

	if (size() == 1) {
		return transforms_.front().transform_;
	}

	//just return the closest
	const auto getMeasurement = std::find_if(transforms_.begin(),
			transforms_.end(), [&time](const TimestampedTransform &tf) {
				return time <= tf.time_;
			});
	const bool isIteratorValid = getMeasurement != transforms_.end();
	if (isIteratorValid && getMeasurement->time_ == time) {
		return getMeasurement->transform_;
	}
	const auto start = std::prev(getMeasurement);

//  std::cout << "buffer size: " << size() << "\n";
//  std::cout << "left time: " << readable(start->time_) << "\n";
//  std::cout << "right time: " << readable(getMeasurement->time_) << "\n";
//  std::cout << "query time: " << readable(time) << "\n \n";
//  std::cout << "times in buffer: \n";
//  printTimesCurrentlyInBuffer();

	return interpolate(*start, *getMeasurement, time).transform_;
}

void TransformInterpolationBuffer::removeOldMeasurementsIfNeeded() {
	while (transforms_.size() > bufferSizeLimit_) {
		transforms_.pop_front();
	}
}

Time TransformInterpolationBuffer::earliest_time() const {
	if (empty()) {
		throw std::runtime_error("TransformInterpolationBuffer:: Empty buffer");
	}
	return transforms_.front().time_;
}

Time TransformInterpolationBuffer::latest_time() const {
	if (empty()) {
		throw std::runtime_error("TransformInterpolationBuffer:: Empty buffer");
	}
	return transforms_.back().time_;
}

bool TransformInterpolationBuffer::empty() const {
	return transforms_.empty();
}

size_t TransformInterpolationBuffer::size_limit() const {
	return bufferSizeLimit_;
}

size_t TransformInterpolationBuffer::size() const {
	return transforms_.size();
}

void TransformInterpolationBuffer::printTimesCurrentlyInBuffer() const {
	for (auto it = transforms_.cbegin(); it != transforms_.cend(); ++it) {
		std::cout << readable(it->time_) << std::endl;
	}
}

Transform getTransform(const Time &time,
		const TransformInterpolationBuffer &buffer) {
	if (time < buffer.earliest_time()) {
		return buffer.lookup(buffer.earliest_time());
	}
	if (time > buffer.latest_time()) {
		return buffer.lookup(buffer.latest_time());
	}
	return buffer.lookup(time);
}

} // namespace m545_mapping


