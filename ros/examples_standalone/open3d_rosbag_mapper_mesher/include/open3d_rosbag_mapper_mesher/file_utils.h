/*
 * file_utils.h
 *
 *  Created on: Nov 19, 2024
 *      Author: nubertj
 */

#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#pragma once

namespace o3d_slam {

std::vector<std::string> getFilesWithPrefix(const std::string& directory, const std::string& prefix) {
  std::vector<std::string> matching_files;

  // Check if the directory exists
  if (!std::filesystem::exists(directory) || !std::filesystem::is_directory(directory)) {
    throw std::runtime_error("Invalid directory: " + directory);
  }

  // Iterate through directory entries
  for (const auto& entry : std::filesystem::directory_iterator(directory)) {
    if (entry.is_regular_file()) {  // Ensure it's a file
      const std::string file_name = entry.path().filename().string();
      if (file_name.rfind(prefix, 0) == 0) {  // Check if the filename starts with the prefix
        matching_files.push_back(entry.path().string());
      }
    }
  }

  // Sort the files
  std::sort(matching_files.begin(), matching_files.end());

  return matching_files;
}

}  // namespace o3d_slam
