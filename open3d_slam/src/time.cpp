/*
 * time.cpp
 *
 *  Created on: Oct 20, 2021
 *      Author: jelavice
 */

#include "open3d_slam/time.hpp"
#include <iostream>
#include <time.h>

#include <cerrno>
#include <cstring>
#include <string>

namespace o3d_slam {

Timer::Timer() :
		Timer(false, "") {

}

Timer::Timer(const std::string &name) :
		Timer(true, name) {
}
Timer::Timer(bool isPrintInDestructor, const std::string &name) {
	startTime_ = std::chrono::steady_clock::now();
	isPrintInDestructor_ = isPrintInDestructor;
	name_ = name;
	startTimeStopWatch_ = startTime_;
}
Timer::~Timer() {
	if (isPrintInDestructor_) {
		std::cout << "Timer " << name_ << ": Elapsed time: " << elapsedMsec() << " msec \n";
	}
}
void Timer::reset() {
	startTime_ = std::chrono::steady_clock::now();
	startTimeStopWatch_ = startTime_;
	numMeasurementsMsec_ = 0;
	cumulativeMeasurementMsec_ = 0.0;
}
void Timer::addMeasurementMsec(double msec) {
	cumulativeMeasurementMsec_ += msec;
	++numMeasurementsMsec_;
}
double Timer::getAvgMeasurementMsec() const {
	return numMeasurementsMsec_ == 0 ? 0.0 : cumulativeMeasurementMsec_ / numMeasurementsMsec_;
}

void Timer::startStopwatch() {
	startTimeStopWatch_ = std::chrono::steady_clock::now();

}
double Timer::elapsedMsecSinceStopwatchStart() const {
	const auto endTime = std::chrono::steady_clock::now();
	return std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTimeStopWatch_).count() / 1e3;
}

double Timer::elapsedMsec() const {
	const auto endTime = std::chrono::steady_clock::now();
	return std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime_).count() / 1e3;
}
double Timer::elapsedSec() const {
	const auto endTime = std::chrono::steady_clock::now();
	return std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime_).count() / 1e3;
}

double readable(const Time &t) {
	return (toUniversal(t) - 637596151404895179) / 10000.0;
}

Duration fromSeconds(const double seconds) {
	return std::chrono::duration_cast<Duration>(std::chrono::duration<double>(seconds));
}

double toSeconds(const Duration duration) {
	return std::chrono::duration_cast<std::chrono::duration<double>>(duration).count();
}

double toSeconds(const std::chrono::steady_clock::duration duration) {
	return std::chrono::duration_cast<std::chrono::duration<double>>(duration).count();
}

Time fromUniversal(const int64 ticks) {
	return Time(Duration(ticks));
}

int64 toUniversal(const Time time) {
	return time.time_since_epoch().count();
}

std::ostream& operator<<(std::ostream &os, const Time time) {
	os << std::to_string(toUniversal(time));
	return os;
}

Duration fromMilliseconds(const int64 milliseconds) {
	return std::chrono::duration_cast<Duration>(std::chrono::milliseconds(milliseconds));
}

std::string toString(const Time &time) {
	return std::to_string(toUniversal(time));
}

ros::Time toRos(Time time) {
	int64_t uts_timestamp = toUniversal(time);
	int64_t ns_since_unix_epoch = (uts_timestamp - kUtsEpochOffsetFromUnixEpochInSeconds * 10000000ll) * 100ll;
	::ros::Time ros_time;
	if (ns_since_unix_epoch < 0){
		std::cerr << "ERROR: nanoseconds since unix epoch is: " << ns_since_unix_epoch << " which is impossible!!!! \n";
		std::cerr << "       ROS time will throw you an exception fo sho!!!! \n";
		std::cerr << "       Are you playing the rosbag with --clock??? \n";
		std::cerr << "       If yes, did you set use_sim_time to true ??? \n";
		std::cout << "Universal time: " << uts_timestamp << std::endl;
	}
	ros_time.fromNSec(ns_since_unix_epoch);
	return ros_time;
}

Time fromRos(const ::ros::Time &time) {
	// The epoch of the ICU Universal Time Scale is "0001-01-01 00:00:00.0 +0000",
	// exactly 719162 days before the Unix epoch.
	return fromUniversal((time.sec + kUtsEpochOffsetFromUnixEpochInSeconds) * 10000000ll + (time.nsec + 50) / 100); // + 50 to get the rounding correct.
}

} /* namespace o3d_slam */
