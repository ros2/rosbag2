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
  const std::filesystem::path & root_bag_path,
  const rosbag2_storage::BagMetadata & metadata,
  int split_index = 0)
{
  if (metadata.files.empty() || split_index >= static_cast<int>(metadata.files.size())) {
    throw std::runtime_error("No file found at split_index " + std::to_string(split_index));
  }
  return root_bag_path / metadata.files[split_index].path;
}

/// Wait for metadata.yaml to appear in bag_path. Throws on timeout.
inline void wait_for_metadata(
  const std::filesystem::path & root_bag_path,
  std::chrono::duration<float> timeout = std::chrono::seconds(5))
{
  rosbag2_storage::MetadataIo metadata_io;
  const auto bag_path_str = root_bag_path.generic_string();
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

/// Count the number of storage files in bag_path.
/// Storage files are identified by their extensions, which are determined by the known storage ids.
inline size_t count_storage_files(const std::filesystem::path & root_bag_path)
{
  size_t count = 0;
  if (std::filesystem::exists(root_bag_path) && std::filesystem::is_directory(root_bag_path)) {
    for (const auto & entry : std::filesystem::directory_iterator(root_bag_path)) {
      if (entry.is_regular_file()) {
        const auto & path = entry.path();
        const auto extension = path.extension().generic_string();
        const auto filename = path.filename().generic_string();

        for (const auto & [storage_id, ext] : kTestedStorageIDsWithExtensions) {
          if (extension == ext || filename.find(ext) != std::string::npos) {
            ++count;
            break;
          }
        }
      }
    }
  }
  return count;
}

/// Wait until at least `expected_count` storage files appear in bag_path. Throws on timeout.
inline void wait_for_storage_files(
  const std::filesystem::path & root_bag_path,
  size_t expected_count = 1,
  std::chrono::duration<float> timeout = std::chrono::seconds(10))
{
  const auto start_time = std::chrono::steady_clock::now();

  while (std::chrono::steady_clock::now() - start_time < timeout) {
    if (count_storage_files(root_bag_path) >= expected_count) {
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  throw std::runtime_error("Timed out waiting for " + std::to_string(expected_count) +
    " storage file(s) in: " + root_bag_path.generic_string());
}

}  // namespace rosbag2_test_common

#endif  // ROSBAG2_TEST_COMMON__BAG_FILES_HELPERS_HPP_
