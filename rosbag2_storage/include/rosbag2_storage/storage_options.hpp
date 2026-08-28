// Copyright 2018, Bosch Software Innovations GmbH.
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

#ifndef ROSBAG2_STORAGE__STORAGE_OPTIONS_HPP_
#define ROSBAG2_STORAGE__STORAGE_OPTIONS_HPP_

#include <cstdint>
#include <string>
#include <unordered_map>

#include "rosbag2_storage/visibility_control.hpp"
#include "rosbag2_storage/yaml.hpp"

namespace rosbag2_storage
{

/// \brief Action to take when the available free space on the filesystem holding the bag falls
/// below the configured minimum (see StorageOptions::min_free_space_bytes and
/// StorageOptions::min_free_space_percent).
enum class LowFreeSpaceAction : uint8_t
{
  /// Stop writing messages to the bag. The recorder will stop the recording.
  STOP = 0,
  /// Delete the oldest bag files (splits) of the current recording until the free space is above
  /// the limit and continue recording. Falls back to STOP if there are no more old bag files to
  /// delete. Only useful when bag splitting is enabled (max_bagfile_size, max_bagfile_duration or
  /// splits triggered via the recorder API or services).
  DELETE_OLDEST_FILES = 1,
};

/// \brief Convert LowFreeSpaceAction to its string representation.
/// \return "stop" or "delete_oldest_files".
ROSBAG2_STORAGE_PUBLIC std::string to_string(LowFreeSpaceAction action);

/// \brief Parse LowFreeSpaceAction from string. Accepted values are "stop" and
/// "delete_oldest_files" (case-insensitive).
/// \throws std::invalid_argument if the string doesn't correspond to any known action.
ROSBAG2_STORAGE_PUBLIC LowFreeSpaceAction low_free_space_action_from_string(
  const std::string & action);

struct StorageOptions
{
public:
  std::string uri;
  std::string storage_id;

  // The maximum size a bagfile can be, in bytes, before it is split.
  // A value of 0 indicates that bagfile splitting will not be used.
  uint64_t max_bagfile_size = 0;

  // The maximum duration a bagfile can be, in seconds, before it is split.
  // A value of 0 indicates that bagfile splitting will not be used.
  uint64_t max_bagfile_duration = 0;

  // Maximum number of bag files to retain before deleting the oldest.
  // A value of 0 disables deletion (unlimited files).
  // This feature is only available when the bag split is active.
  // Requires --max-bag-size or --max-bag-duration to be set or usage of the bag split via
  // direct recorder API or service calls.
  uint64_t max_bag_files = 0;

  // The cache size. Indicates how many messages can maximally be held in cache before these being
  // written to disk. Works together with max_cache_duration bound if set.
  // A value of 0 disables size-based caching and every write happens directly to disk if
  // max_cache_duration is also set to 0.
  uint64_t max_cache_size = 0;

  // Maximum cache duration in seconds. Used for time-limited buffering (applies to both snapshot
  // mode and regular caching). A value of 0 indicates that buffering will be limited by the
  // max_cache_size only. When greater than 0, the cache buffer maintains messages within
  // this time window and drops newer messages if cache overflow happened.
  // Works together with max_cache_size bound if set.
  uint32_t max_cache_duration = 0;

  // Preset storage configuration. Preset settings can be overriden with
  // corresponding settings specified through storage_config_uri file
  std::string storage_preset_profile = "";

  // Storage specific configuration file.
  // Defaults to empty string.
  std::string storage_config_uri = "";

  // Enable snapshot mode.
  // Defaults to disabled.
  bool snapshot_mode = false;

  // Start and end time for cutting. Used in the writers to limit the range of stored messages.
  // As well as in the "ros2 bag convert" CLI aka "bag_rewrite" utility to limit the range of the
  // reading and writing messages.
  int64_t start_time_ns = -1;
  int64_t end_time_ns = -1;

  // Stores the custom data
  std::unordered_map<std::string, std::string> custom_data{};

  // Note: New fields are appended at the end to keep positional aggregate initialization of
  // StorageOptions in existing code working.

  // Minimum free space in bytes which shall remain available on the filesystem where the bag is
  // being written. When the available free space goes below this limit, the
  // low_free_space_action is taken. A value of 0 disables the check.
  // Note: This is a bound on the whole filesystem, not on the bag size. It protects against the
  // disk being filled by the recording and by anything else growing on the same filesystem.
  uint64_t min_free_space_bytes = 0;

  // Minimum free space as a percentage (0.0 - 100.0) of the filesystem capacity which shall
  // remain available. When the available free space goes below this limit, the
  // low_free_space_action is taken. A value of 0 disables the check.
  // If both min_free_space_bytes and min_free_space_percent are set, the larger resulting
  // threshold in bytes is used.
  double min_free_space_percent = 0.0;

  // Action to take when the available free space goes below the configured minimum.
  // Defaults to stopping the recording.
  LowFreeSpaceAction low_free_space_action = LowFreeSpaceAction::STOP;
};

}  // namespace rosbag2_storage

namespace YAML
{
template<>
struct ROSBAG2_STORAGE_PUBLIC convert<rosbag2_storage::StorageOptions>
{
  static Node encode(const rosbag2_storage::StorageOptions & storage_options);
  static bool decode(const Node & node, rosbag2_storage::StorageOptions & storage_options);
};
}  // namespace YAML

#endif  // ROSBAG2_STORAGE__STORAGE_OPTIONS_HPP_
