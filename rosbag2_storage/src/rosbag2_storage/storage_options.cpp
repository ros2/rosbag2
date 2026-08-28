// Copyright 2021 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#include <algorithm>
#include <cctype>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include "rosbag2_storage/storage_options.hpp"

namespace rosbag2_storage
{

std::string to_string(LowFreeSpaceAction action)
{
  switch (action) {
    case LowFreeSpaceAction::STOP:
      return "stop";
    case LowFreeSpaceAction::DELETE_OLDEST_FILES:
      return "delete_oldest_files";
    default:
      throw std::invalid_argument(
              "Unknown LowFreeSpaceAction value: " +
              std::to_string(static_cast<int>(action)));
  }
}

LowFreeSpaceAction low_free_space_action_from_string(const std::string & action)
{
  std::string lower_case_action = action;
  std::transform(
    lower_case_action.begin(), lower_case_action.end(), lower_case_action.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  if (lower_case_action == "stop") {
    return LowFreeSpaceAction::STOP;
  }
  if (lower_case_action == "delete_oldest_files") {
    return LowFreeSpaceAction::DELETE_OLDEST_FILES;
  }
  throw std::invalid_argument(
          "Unknown low free space action: '" + action +
          "'. Expected one of: 'stop', 'delete_oldest_files'.");
}

}  // namespace rosbag2_storage

namespace YAML
{

template<>
struct convert<rosbag2_storage::LowFreeSpaceAction>
{
  static Node encode(const rosbag2_storage::LowFreeSpaceAction & action)
  {
    Node node;
    node = rosbag2_storage::to_string(action);
    return node;
  }

  static bool decode(const Node & node, rosbag2_storage::LowFreeSpaceAction & action)
  {
    if (!node.IsScalar()) {
      return false;
    }
    try {
      action = rosbag2_storage::low_free_space_action_from_string(node.as<std::string>());
    } catch (const std::invalid_argument & ex) {
      throw YAML::Exception(node.Mark(), ex.what());
    }
    return true;
  }
};

Node convert<rosbag2_storage::StorageOptions>::encode(
  const rosbag2_storage::StorageOptions & storage_options)
{
  Node node;
  node["uri"] = storage_options.uri;
  node["storage_id"] = storage_options.storage_id;
  node["max_bagfile_size"] = storage_options.max_bagfile_size;
  node["max_bagfile_duration"] = storage_options.max_bagfile_duration;
  node["max_bag_files"] = storage_options.max_bag_files;
  node["min_free_space_bytes"] = storage_options.min_free_space_bytes;
  node["min_free_space_percent"] = storage_options.min_free_space_percent;
  node["low_free_space_action"] = storage_options.low_free_space_action;
  node["max_cache_size"] = storage_options.max_cache_size;
  node["max_cache_duration"] = storage_options.max_cache_duration;
  node["storage_preset_profile"] = storage_options.storage_preset_profile;
  node["storage_config_uri"] = storage_options.storage_config_uri;
  node["snapshot_mode"] = storage_options.snapshot_mode;
  node["start_time_ns"] = storage_options.start_time_ns;
  node["end_time_ns"] = storage_options.end_time_ns;
  node["custom_data"] = storage_options.custom_data;
  return node;
}

bool convert<rosbag2_storage::StorageOptions>::decode(
  const Node & node, rosbag2_storage::StorageOptions & storage_options)
{
  if (node["uri"] && !node["uri"].IsNull()) {
    storage_options.uri = node["uri"].as<std::string>();
  }
  optional_assign<std::string>(node, "storage_id", storage_options.storage_id);
  optional_assign<uint64_t>(node, "max_bagfile_size", storage_options.max_bagfile_size);
  optional_assign<uint64_t>(node, "max_bagfile_duration", storage_options.max_bagfile_duration);
  optional_assign<uint64_t>(node, "max_bag_files", storage_options.max_bag_files);
  optional_assign<uint64_t>(
    node, "min_free_space_bytes", storage_options.min_free_space_bytes);
  optional_assign<double>(
    node, "min_free_space_percent", storage_options.min_free_space_percent);
  optional_assign<rosbag2_storage::LowFreeSpaceAction>(
    node, "low_free_space_action", storage_options.low_free_space_action);
  optional_assign<uint64_t>(node, "max_cache_size", storage_options.max_cache_size);
  optional_assign<uint32_t>(node, "max_cache_duration", storage_options.max_cache_duration);
  optional_assign<std::string>(
    node, "storage_preset_profile", storage_options.storage_preset_profile);
  optional_assign<std::string>(node, "storage_config_uri", storage_options.storage_config_uri);
  optional_assign<bool>(node, "snapshot_mode", storage_options.snapshot_mode);
  optional_assign<int64_t>(node, "start_time_ns", storage_options.start_time_ns);
  optional_assign<int64_t>(node, "end_time_ns", storage_options.end_time_ns);
  using KEY_VALUE_MAP = std::unordered_map<std::string, std::string>;
  optional_assign<KEY_VALUE_MAP>(node, "custom_data", storage_options.custom_data);
  return true;
}

}  // namespace YAML
