// Copyright 2026 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROSBAG2_TEST_COMMON__BAG_FILES_HELPERS_HPP_
#define ROSBAG2_TEST_COMMON__BAG_FILES_HELPERS_HPP_

#include <chrono>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <thread>

#include "rosbag2_storage/bag_metadata.hpp"
#include "rosbag2_storage/metadata_io.hpp"

#include "rosbag2_test_common/tested_storage_ids.hpp"

namespace rosbag2_test_common
{

/// Get the full path to a bag file using already-loaded metadata.
inline std::filesystem::path get_bag_file_path_from_metadata(
  const std::filesystem::path & bag_path,
  const rosbag2_storage::BagMetadata & metadata,
  int split_index = 0)
{
  if (metadata.files.empty() || split_index >= static_cast<int>(metadata.files.size())) {
    throw std::runtime_error("No file found at split_index " + std::to_string(split_index));
  }
  return bag_path / metadata.files[split_index].path;
}

/// Get the full path to a compressed bag file using already-loaded metadata.
inline std::filesystem::path get_compressed_bag_file_path(
  const std::filesystem::path & bag_path,
  const rosbag2_storage::BagMetadata & metadata,
  int split_index = 0)
{
  return std::filesystem::path(
    get_bag_file_path_from_metadata(bag_path, metadata, split_index).generic_string() + ".zstd");
}

/// Get the relative path for a bag file in the old naming format (name_index.ext).
inline std::filesystem::path get_relative_bag_file_path(
  const std::string & bag_name,
  int split_index,
  const std::string & storage_id)
{
  std::stringstream bag_file_name;
  bag_file_name << bag_name << "_" << split_index;
  return std::filesystem::path(bag_filename_for_storage_id(bag_file_name.str(), storage_id));
}

/// Wait for metadata.yaml to appear in bag_path. Throws on timeout.
inline void wait_for_metadata(
  const std::filesystem::path & bag_path,
  std::chrono::duration<float> timeout = std::chrono::seconds(5))
{
  rosbag2_storage::MetadataIo metadata_io;
  const auto bag_path_str = bag_path.generic_string();
  const auto start_time = std::chrono::steady_clock::now();

  while (std::chrono::steady_clock::now() - start_time < timeout) {
    if (metadata_io.metadata_file_exists(bag_path_str)) {
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  if (!metadata_io.metadata_file_exists(bag_path_str)) {
    throw std::runtime_error("Could not find metadata file for bag: " + bag_path_str);
  }
}

/// Wait for any storage file (.db3 or .mcap) to appear in bag_path. Throws on timeout.
inline void wait_for_storage_file(
  const std::filesystem::path & bag_path,
  std::chrono::duration<float> timeout = std::chrono::seconds(10))
{
  const auto start_time = std::chrono::steady_clock::now();

  while (std::chrono::steady_clock::now() - start_time < timeout) {
    if (std::filesystem::exists(bag_path) && std::filesystem::is_directory(bag_path)) {
      for (const auto & entry : std::filesystem::directory_iterator(bag_path)) {
        if (entry.is_regular_file()) {
          const auto & path = entry.path();
          const auto extension = path.extension();
          if (extension == ".db3" || extension == ".mcap" ||
            path.filename().generic_string().find(".db3.") != std::string::npos ||
            path.filename().generic_string().find(".mcap.") != std::string::npos)
          {
            return;
          }
        }
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  throw std::runtime_error(
    "Could not find any storage file in directory: " + bag_path.generic_string());
}

}  // namespace rosbag2_test_common

#endif  // ROSBAG2_TEST_COMMON__BAG_FILES_HELPERS_HPP_
