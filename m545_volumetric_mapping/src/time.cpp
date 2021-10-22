/*
 * time.cpp
 *
 *  Created on: Oct 20, 2021
 *      Author: jelavice
 */



#include "m545_volumetric_mapping/time.hpp"
#include <iostream>


namespace m545_mapping {

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
}
Timer::~Timer() {
	if (isPrintInDestructor_) {
		std::cout << "Timer " << name_ << ": Elapsed time: " << elapsedMsec() << " msec \n";
	}
}
void Timer::reset(){
	startTime_ = std::chrono::steady_clock::now();
}
double Timer::elapsedMsec() const {
	const auto endTime = std::chrono::steady_clock::now();
	return std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime_).count() / 1e3;
}
double Timer::elapsedSec() const {
	const auto endTime = std::chrono::steady_clock::now();
	return std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime_).count() / 1e3;
}

} /* namespace m545_mapping */
