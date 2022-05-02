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

bool Timer::isDisablePrintInDestructor_ = false;

void updateFirstMeasurementTime(const Time &t) {
	if (!time_internal::isFirstMeasurementTimeUpdated) {
		time_internal::firstMeasurementTime = t;
		time_internal::isFirstMeasurementTimeUpdated = true;
	}
}

bool isTimeValid(const Time &t){
	return toUniversal(t) > 0;
}

double toSecondsSinceFirstMeasurement(const Time &t) {
	return
			time_internal::isFirstMeasurementTimeUpdated ? toSeconds(t - time_internal::firstMeasurementTime) : 0.0;
}

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
	if (!isDisablePrintInDestructor_ && isPrintInDestructor_) {
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

} /* namespace o3d_slam */
