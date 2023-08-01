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

#include <string>
#include <unordered_map>

#include "rosbag2_storage/storage_options.hpp"

namespace rosbag2_storage
{

namespace
{
rcl_interfaces::msg::ParameterDescriptor int_param_description(
  std::string description, int64_t min,
  int64_t max)
{
  rcl_interfaces::msg::ParameterDescriptor d{};
  rcl_interfaces::msg::IntegerRange r{};
  d.description = std::move(description);
  r.from_value = min;
  r.to_value = max;
  d.integer_range.push_back(r);
  return d;
}
}  // namespace

void init_storage_options_from_node_params(
  std::shared_ptr<rclcpp::Node> node,
  StorageOptions & storage_options)
{
  storage_options.uri = node->declare_parameter<std::string>(
    "uri",
    "");

  storage_options.storage_id = node->declare_parameter<std::string>(
    "storage_id",
    "");

  storage_options.storage_config_uri = node->declare_parameter<std::string>(
    "config_uri",
    "");

  auto desc_mbs = int_param_description(
    "Max bagfile size (bytes)",
    1,
    std::numeric_limits<int64_t>::max());
  auto max_bagfile_size_ = node->declare_parameter<int64_t>(
    "max_bagfile_size",
    0,
    desc_mbs);
  storage_options.max_bagfile_size = static_cast<uint64_t>(max_bagfile_size_);

  auto desc_mbd = int_param_description(
    "Max bagfile duration (nanoseconds)",
    1,
    std::numeric_limits<int64_t>::max());
  auto max_bagfile_duration_ = node->declare_parameter<int64_t>(
    "max_bagfile_duration",
    0,
    desc_mbd);
  storage_options.max_bagfile_duration = static_cast<uint64_t>(max_bagfile_duration_);

  auto desc_mcs = int_param_description(
    "Max chache size (messages)",
    1,
    std::numeric_limits<int64_t>::max());
  auto max_cache_size_ = node->declare_parameter<int64_t>(
    "max_cache_size",
    0,
    desc_mcs);
  storage_options.max_cache_size = static_cast<uint64_t>(max_cache_size_);

  storage_options.storage_preset_profile = node->declare_parameter<std::string>(
    "preset_profile",
    "");

  storage_options.snapshot_mode = node->declare_parameter<bool>(
    "snapshot_mode",
    false);

  auto list_of_key_value_strings = node->declare_parameter<std::vector<std::string>>(
    "custom_data",
    std::vector<std::string>());
  for (const auto & key_value_string : list_of_key_value_strings) {
    auto delimiter_pos = key_value_string.find("=", 0);
    if (delimiter_pos == std::string::npos) {
      std::stringstream ss;
      ss << "The storage.custom_data expected to be as list of the key=value strings. "
        "The `=` not found in the " << key_value_string;
      throw std::invalid_argument(ss.str());
    }
    auto key_string = key_value_string.substr(0, delimiter_pos);
    auto value_string = key_value_string.substr(delimiter_pos + 1);
    storage_options.custom_data[key_string] = value_string;
  }
}
}  // namespace rosbag2_storage

namespace YAML
{

Node convert<rosbag2_storage::StorageOptions>::encode(
  const rosbag2_storage::StorageOptions & storage_options)
{
  Node node;
  node["uri"] = storage_options.uri;
  node["storage_id"] = storage_options.storage_id;
  node["max_bagfile_size"] = storage_options.max_bagfile_size;
  node["max_bagfile_duration"] = storage_options.max_bagfile_duration;
  node["max_cache_size"] = storage_options.max_cache_size;
  node["storage_preset_profile"] = storage_options.storage_preset_profile;
  node["storage_config_uri"] = storage_options.storage_config_uri;
  node["snapshot_mode"] = storage_options.snapshot_mode;
  node["custom_data"] = storage_options.custom_data;
  return node;
}

bool convert<rosbag2_storage::StorageOptions>::decode(
  const Node & node, rosbag2_storage::StorageOptions & storage_options)
{
  storage_options.uri = node["uri"].as<std::string>();
  optional_assign<std::string>(node, "storage_id", storage_options.storage_id);
  optional_assign<uint64_t>(node, "max_bagfile_size", storage_options.max_bagfile_size);
  optional_assign<uint64_t>(node, "max_bagfile_duration", storage_options.max_bagfile_duration);
  optional_assign<uint64_t>(node, "max_cache_size", storage_options.max_cache_size);
  optional_assign<std::string>(
    node, "storage_preset_profile", storage_options.storage_preset_profile);
  optional_assign<std::string>(node, "storage_config_uri", storage_options.storage_config_uri);
  optional_assign<bool>(node, "snapshot_mode", storage_options.snapshot_mode);
  using KEY_VALUE_MAP = std::unordered_map<std::string, std::string>;
  optional_assign<KEY_VALUE_MAP>(node, "custom_data", storage_options.custom_data);
  return true;
}

}  // namespace YAML
