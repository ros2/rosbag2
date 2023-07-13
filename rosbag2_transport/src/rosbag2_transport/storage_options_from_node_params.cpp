// Copyright 2023 Open Source Robotics Foundation, Inc.
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

#include "rosbag2_transport/utils/param_utils.hpp"
#include "rosbag2_transport/storage_options_from_node_params.hpp"

namespace rosbag2_transport
{

rosbag2_storage::StorageOptions
get_storage_options_from_node_params(std::shared_ptr<rclcpp::Node> node)
{
  rosbag2_storage::StorageOptions storage_options{};

  storage_options.uri = node->declare_parameter<std::string>("uri", "");

  storage_options.storage_id = node->declare_parameter<std::string>("storage_id", "");

  storage_options.storage_config_uri = node->declare_parameter<std::string>("config_uri", "");

  auto desc_mbs = param_utils::int_param_description(
    "Max bagfile size (bytes)",
    1,
    std::numeric_limits<int64_t>::max());
  auto max_bagfile_size_ = node->declare_parameter<int64_t>(
    "max_bagfile_size",
    0,
    desc_mbs);
  storage_options.max_bagfile_size = static_cast<uint64_t>(max_bagfile_size_);

  auto desc_mbd = param_utils::int_param_description(
    "Max bagfile duration (nanoseconds)",
    1,
    std::numeric_limits<int64_t>::max());
  auto max_bagfile_duration_ = node->declare_parameter<int64_t>(
    "max_bagfile_duration",
    0,
    desc_mbd);
  storage_options.max_bagfile_duration = static_cast<uint64_t>(max_bagfile_duration_);

  auto desc_mcs = param_utils::int_param_description(
    "Max chache size (messages)",
    1,
    std::numeric_limits<int64_t>::max());
  auto max_cache_size_ = node->declare_parameter<int64_t>(
    "max_cache_size",
    0,
    desc_mcs);
  storage_options.max_cache_size = static_cast<uint64_t>(max_cache_size_);

  storage_options.storage_preset_profile =
    node->declare_parameter<std::string>("preset_profile", "");

  storage_options.snapshot_mode = node->declare_parameter<bool>("snapshot_mode", false);

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

  return storage_options;
}

}  // namespace rosbag2_transport
