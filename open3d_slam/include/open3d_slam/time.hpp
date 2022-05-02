/*
 * time.hpp
 *
 *  Created on: Oct 20, 2021
 *      Author: jelavice
 */

#pragma once
#include <chrono>
#include <string>
#include <chrono>
#include <ostream>
#include <ratio>
#include "open3d_slam/typedefs.hpp"

namespace o3d_slam {

class Timer {
public:
	Timer();
	Timer(const std::string &name);
	Timer(bool isPrintInDestructor, const std::string &name);
	~Timer();
	double elapsedMsec() const;
	double elapsedSec() const;
	void reset();
	void startStopwatch();
	double elapsedMsecSinceStopwatchStart() const;
	void addMeasurementMsec(double msec);
	double getAvgMeasurementMsec() const;
	static bool isDisablePrintInDestructor_;
private:
	std::chrono::steady_clock::time_point startTime_;
	std::chrono::steady_clock::time_point startTimeStopWatch_;
	bool isPrintInDestructor_ = false;
	std::string name_;
	double cumulativeMeasurementMsec_ = 0.0;
	size_t numMeasurementsMsec_ = 0;
};



constexpr int64 kUtsEpochOffsetFromUnixEpochInSeconds =
    (719162ll * 24ll * 60ll * 60ll);

struct UniversalTimeScaleClock {
  using rep = int64;
  using period = std::ratio<1, 10000000>; // 100 / 1000000000
  using duration = std::chrono::duration<rep, period>;
  using time_point = std::chrono::time_point<UniversalTimeScaleClock>;
  static constexpr bool is_steady = true;
};

// Represents Universal Time Scale durations and timestamps which are 64-bit
// integers representing the 100 nanosecond ticks since the Epoch which is
// January 1, 1 at the start of day in UTC.
using Duration = UniversalTimeScaleClock::duration;
using Time = UniversalTimeScaleClock::time_point;

// Convenience functions to create common::Durations.
Duration fromSeconds(double seconds);
Duration fromMilliseconds(int64 milliseconds);

// Returns the given duration in seconds.
double toSeconds(Duration duration);
double toSeconds(std::chrono::steady_clock::duration duration);

// Creates a time from a Universal Time Scale.
Time fromUniversal(int64 ticks);

// Outputs the Universal Time Scale timestamp for a given Time.
int64 toUniversal(Time time);

// For logging and unit tests, outputs the timestamp integer.
std::ostream& operator<<(std::ostream& os, Time time);

std::string toString(const Time &time);

void updateFirstMeasurementTime(const Time &t);
double toSecondsSinceFirstMeasurement(const Time &t);
bool isTimeValid(const Time &t);


namespace time_internal {
static Time firstMeasurementTime;
static bool isFirstMeasurementTimeUpdated = false;
}


} /* namespace o3d_slam */
