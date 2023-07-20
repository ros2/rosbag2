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
  d.description = description;
  r.from_value = min;
  r.to_value = max;
  d.integer_range.push_back(r);
  return d;
}
}  // namespace

void declare_storage_options_r_params(std::shared_ptr<rclcpp::Node> nh, StorageOptions & so)
{
  // TODO(roncapat): check if file can be read
  so.uri = nh->declare_parameter<std::string>(
    "storage.uri",
    "");

  so.storage_id = nh->declare_parameter<std::string>(
    "storage.storage_id",
    "");

  // TODO(roncapat): check if file can be read
  so.storage_config_uri = nh->declare_parameter<std::string>(
    "storage.config_uri",
    "");
}

void declare_storage_options_rw_params(std::shared_ptr<rclcpp::Node> nh, StorageOptions & so)
{
  auto desc_mbs = int_param_description(
    "Max bagfile size (bytes)",
    1,
    std::numeric_limits<int64_t>::max());
  auto max_bagfile_size_ = nh->declare_parameter<int64_t>(
    "storage.max_bagfile_size",
    0,
    desc_mbs);
  so.max_bagfile_size = static_cast<uint64_t>(max_bagfile_size_);

  auto desc_mbd = int_param_description(
    "Max bagfile duration (nanoseconds)",
    1,
    std::numeric_limits<int64_t>::max());
  auto max_bagfile_duration_ = nh->declare_parameter<int64_t>(
    "storage.max_bagfile_duration",
    0,
    desc_mbd);
  so.max_bagfile_duration = static_cast<uint64_t>(max_bagfile_duration_);

  auto desc_mcs = int_param_description(
    "Max chache size (messages)",
    1,
    std::numeric_limits<int64_t>::max());
  auto max_cache_size_ = nh->declare_parameter<int64_t>(
    "storage.max_cache_size",
    0,
    desc_mcs);
  so.max_cache_size = static_cast<uint64_t>(max_cache_size_);

  so.storage_preset_profile = nh->declare_parameter<std::string>(
    "storage.preset_profile",
    "");

  so.snapshot_mode = nh->declare_parameter<bool>(
    "storage.snapshot_mode",
    false);

  // FIXME(roncapat)
  // std::unordered_map<std::string, std::string> custom_data{};
  declare_storage_options_r_params(nh, so);
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
