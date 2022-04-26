/*
 * TransformInterpolationBuffer.hpp
 *
 *  Created on: Nov 8, 2021
 *      Author: jelavice
 */

#pragma once
#include <deque>
#include <limits>
#include <mutex>

#include "open3d_slam/time.hpp"
#include "open3d_slam/Transform.hpp"

namespace o3d_slam {

// A time-ordered buffer of transforms that supports interpolated lookups.
// Unless explicitly set, the buffer size is unlimited.
class TransformInterpolationBuffer {
public:
	TransformInterpolationBuffer(size_t bufferSize);
	TransformInterpolationBuffer();
	virtual ~TransformInterpolationBuffer() = default;

	// Sets the transform buffer size limit and removes old transforms
	// if it is exceeded.
	void setSizeLimit(size_t bufferSizeLimit);

	// Adds a new transform to the buffer and removes the oldest transform if the
	// buffer size limit is exceeded.
	void push(const Time &time, const Transform &transform);

	// Clears the transform buffer.
	void clear();

	// Returns true if an interpolated transform can be computed at 'time'.
	bool has(const Time &time) const;

	// Returns an interpolated transform at 'time'. CHECK()s that a transform at
	// 'time' is available.
	Transform lookup(const Time &time) const;

	// Returns the timestamp of the earliest transform in the buffer or 0 if the
	// buffer is empty. Earliest time is the one that is the closest to Jan 1,1,00
	Time earliest_time() const;

	// Returns the timestamp of the earliest transform in the buffer or 0 if the
	// buffer is empty. Latest time is the one that is the farthest from Jan 1,1,00
	Time latest_time() const;

	// Returns true if the buffer is empty.
	bool empty() const;

	// Returns the maximum allowed size of the transform buffer.
	size_t size_limit() const;

	// Returns the current size of the transform buffer.
	size_t size() const;

	const TimestampedTransform& latest_measurement(int offsetFromLastElement = 0) const;
	TimestampedTransform& latest_measurement(int offsetFromLastElement = 0);

	void printTimesCurrentlyInBuffer() const;

	void applyToAllElementsInTimeInterval(const Transform &t, const Time &begin, const Time &end );

private:
	void removeOldMeasurementsIfNeeded();
	static constexpr size_t kUnlimitedBufferSize = std::numeric_limits<size_t>::max();

	std::deque<TimestampedTransform> transforms_;
	size_t bufferSizeLimit_ = kUnlimitedBufferSize;
	mutable std::mutex modifierMutex_;
};

Transform getTransform(const Time &time,
		const TransformInterpolationBuffer &buffer);

} // namespace o3d_slam
