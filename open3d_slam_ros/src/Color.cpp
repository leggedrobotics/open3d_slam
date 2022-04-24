/*
 * Color.cpp
 *
 *  Created on: Apr 21, 2022
 *      Author: jelavice
 */



#include "open3d_slam_ros/Color.hpp"


namespace o3d_slam {

namespace {
template<typename T, typename Limits>
void clamp(T *val, Limits lo, Limits hi) {
	if (*val > hi) {
		*val = hi;
		return;
	}
	if (*val < lo) {
		*val = lo;
		return;
	}
}
} // namespace

Color::Color() :
		std_msgs::ColorRGBA() {
}
Color::Color(double red, double green, double blue) :
		Color(red, green, blue, 1.0) {
}
Color::Color(double red, double green, double blue, double alpha) :
		Color() {
	r = red;
	g = green;
	b = blue;
	a = alpha;
}

Color Color::operator*(double scalar) const {
	Color ret = *this;
	ret.r *= scalar;
	ret.g *= scalar;
	ret.b *= scalar;
	clamp(&ret.r, 0.0, 1.0);
	clamp(&ret.g, 0.0, 1.0);
	clamp(&ret.b, 0.0, 1.0);
	return ret;
}
Color operator*(double scalar, const Color &c) {
	return c * scalar;
}
const Color Color::getColor(int colorCode) {

	switch (colorCode) {
	case 0: {
		return White();
	}
	case 1: {
		return Black();
	}
	case 2: {
		return Gray();
	}
	case 3: {
		return Red();
	}
	case 4: {
		return Green();
	}
	case 5: {
		return Blue();
	}
	case 6: {
		return Yellow();
	}
	case 7: {
		return Orange();
	}
	case 8: {
		return Purple();
	}
	case 9: {
		return Chartreuse();
	}
	case 10: {
		return Teal();
	}
	case 11: {
		return Pink();
	}
	case 12: {
		return Magenta();
	}
	default:
		throw std::runtime_error("unknown color code");
	}

}

} /* namespace o3d_slam */
