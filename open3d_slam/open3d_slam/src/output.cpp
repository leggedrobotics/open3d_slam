/*
 * output.cpp
 *
 *  Created on: Feb 21, 2022
 *      Author: jelavice
 */

#include "open3d_slam/output.hpp"
#include "open3d_slam/math.hpp"

#include <open3d/io/PointCloudIO.h>
#include <Eigen/Dense>
#include <algorithm>
#include <cctype>
#include <filesystem>
#include <memory>

namespace o3d_slam {

namespace {

std::string toLower(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value;
}

std::filesystem::path replaceOrAppendExtension(const std::filesystem::path& path, const std::string& extension) {
  auto updated = path;
  if (updated.has_extension()) {
    updated.replace_extension(extension);
  } else {
    updated += extension;
  }
  return updated;
}

bool writePointCloud(const std::filesystem::path& path, const PointCloud& cloud) {
  PointCloud copy = cloud;
  return open3d::io::WritePointCloud(path.string(), copy, open3d::io::WritePointCloudOption());
}

bool writeLegacyPcd(const std::filesystem::path& path, const PointCloud& cloud) {
  PointCloud copy = cloud;
  return open3d::io::WritePointCloudToPCD(path.string(), copy, open3d::io::WritePointCloudOption());
}

void removeIfExists(const std::filesystem::path& path) {
  std::error_code ec;
  std::filesystem::remove(path, ec);
}

}  // namespace

std::string asString(const Transform& T) {
  const double kRadToDeg = 180.0 / M_PI;
  const auto& t = T.translation();
  const auto& q = Eigen::Quaterniond(T.rotation());
  const std::string trans = string_format("t:[%f, %f, %f]", t.x(), t.y(), t.z());
  const std::string rot = string_format("q:[%f, %f, %f, %f]", q.x(), q.y(), q.z(), q.w());
  const auto rpy = toRPY(q) * kRadToDeg;
  const std::string rpyString = string_format("rpy (deg):[%f, %f, %f]", rpy.x(), rpy.y(), rpy.z());
  return trans + " ; " + rot + " ; " + rpyString;
}

std::string asStringXYZRPY(const Transform& T) {
  const double kRadToDeg = 180.0 / M_PI;
  const auto& t = T.translation();
  const auto& q = Eigen::Quaterniond(T.rotation());
  const std::string trans = string_format("t:[%f, %f, %f]", t.x(), t.y(), t.z());
  const auto rpy = toRPY(q) * kRadToDeg;
  const std::string rpyString = string_format("rpy (deg):[%f, %f, %f]", rpy.x(), rpy.y(), rpy.z());
  return trans + " ; " + rpyString;
}

bool saveToFile(const std::string& filename, const PointCloud& cloud) {
  const std::filesystem::path requestedPath(filename);
  const std::string extension = toLower(requestedPath.extension().string());

  if (cloud.HasColors()) {
    if (extension.empty() || extension == ".pcd") {
      const auto targetPath = replaceOrAppendExtension(requestedPath, ".ply");
      const bool success = writePointCloud(targetPath, cloud);
      if (success) {
        removeIfExists(replaceOrAppendExtension(requestedPath, ".pcd"));
      }
      return success;
    }
    return writePointCloud(requestedPath, cloud);
  }

  if (extension.empty()) {
    const auto targetPath = replaceOrAppendExtension(requestedPath, ".pcd");
    const bool success = writeLegacyPcd(targetPath, cloud);
    if (success) {
      removeIfExists(replaceOrAppendExtension(requestedPath, ".ply"));
    }
    return success;
  }
  if (extension == ".pcd") {
    const bool success = writeLegacyPcd(requestedPath, cloud);
    if (success) {
      removeIfExists(replaceOrAppendExtension(requestedPath, ".ply"));
    }
    return success;
  }
  return writePointCloud(requestedPath, cloud);
}

bool createDirectoryOrNoActionIfExists(const std::string& directory) {
  return std::filesystem::create_directories(directory);
}

}  // namespace o3d_slam
