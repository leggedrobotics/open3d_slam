/*
 * output.hpp
 *
 *  Created on: Sep 28, 2021
 *      Author: jelavice
 */

#pragma once
#include <string>
#include <memory>
#include "open3d_slam/Transform.hpp"

namespace o3d_slam {

template<typename ... Args>
std::string string_format( const std::string& format, Args ... args )
{
    using namespace std;
    int size_s = snprintf( nullptr, 0, format.c_str(), args ... ) + 1; // Extra space for '\0'
    if( size_s <= 0 ){ throw std::runtime_error( "Error during formatting." ); }
    auto size = static_cast<size_t>( size_s );
    auto buf = make_unique<char[]>( size );
    snprintf( buf.get(), size, format.c_str(), args ... );
    return std::string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
}


std::string asString(const Transform &T);
std::string asStringXYZRPY(const Transform &T);
bool saveToFile(const std::string &filename, const PointCloud &cloud);

//outputs true if directory has been created
bool createDirectoryOrNoActionIfExists(const std::string &directory);

} // namespace o3d_slam
