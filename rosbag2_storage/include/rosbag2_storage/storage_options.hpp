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
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>

#include "rclcpp/node.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/integer_range.hpp"
#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rosbag2_storage/visibility_control.hpp"
#include "rosbag2_storage/yaml.hpp"

namespace rosbag2_storage
{

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

  // The cache size indiciates how many messages can maximally be hold in cache
  // before these being written to disk.
  // A value of 0 disables caching and every write happens directly to disk.
  uint64_t max_cache_size = 0;

  // Preset storage configuration. Preset settings can be overriden with
  // corresponding settings specified through storage_config_uri file
  std::string storage_preset_profile = "";

  // Storage specific configuration file.
  // Defaults to empty string.
  std::string storage_config_uri = "";

  // Enable snapshot mode.
  // Defaults to disabled.
  bool snapshot_mode = false;

  // Stores the custom data
  std::unordered_map<std::string, std::string> custom_data{};
};

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

rcl_interfaces::msg::ParameterDescriptor float_param_description(
  std::string description, float min,
  float max)
{
  rcl_interfaces::msg::ParameterDescriptor d{};
  rcl_interfaces::msg::FloatingPointRange r{};
  d.description = description;
  r.from_value = min;
  r.to_value = max;
  d.floating_point_range.push_back(r);
  return d;
}
}  // namespace

void declare_storage_options_r_params(std::shared_ptr<rclcpp::Node> nh, Storageoptions & so)
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

void declare_storage_options_rw_params(std::shared_ptr<rclcpp::Node> nh, Storageoptions & so)
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
template<>
struct ROSBAG2_STORAGE_PUBLIC convert<rosbag2_storage::StorageOptions>
{
  static Node encode(const rosbag2_storage::StorageOptions & so);
  static bool decode(const Node & node, rosbag2_storage::StorageOptions & so);
};
}  // namespace YAML

#endif  // ROSBAG2_STORAGE__STORAGE_OPTIONS_HPP_
