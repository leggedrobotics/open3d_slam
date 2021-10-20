/*
 * time.hpp
 *
 *  Created on: Oct 20, 2021
 *      Author: jelavice
 */

#pragma once
#include <chrono>
#include <string>


namespace m545_mapping {

class Timer {
public:
	Timer();
	Timer(const std::string &name);
	Timer(bool isPrintInDestructor, const std::string &name);
	~Timer();
	double elapsedMsec() const;
	double elapsedSec() const;
private:
	std::chrono::steady_clock::time_point startTime_;
	bool isPrintInDestructor_ = false;
	std::string name_;
};

} /* namespace m545_mapping */
