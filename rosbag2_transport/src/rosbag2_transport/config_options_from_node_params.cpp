// Copyright 2023 Patrick Roncagliolo and Michael Orlov.
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
#include <vector>

#include "rclcpp/logging.hpp"
#include "rosbag2_storage/qos.hpp"
#include "rosbag2_transport/play_options.hpp"
#include "rosbag2_transport/config_options_from_node_params.hpp"

namespace rosbag2_transport
{

namespace param_utils
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

rcl_interfaces::msg::ParameterDescriptor float_param_description(
  std::string description, float min,
  float max)
{
  rcl_interfaces::msg::ParameterDescriptor d{};
  rcl_interfaces::msg::FloatingPointRange r{};
  d.description = std::move(description);
  r.from_value = min;
  r.to_value = max;
  d.floating_point_range.push_back(r);
  return d;
}

template<typename T>
typename std::enable_if<std::numeric_limits<T>::is_integer, T>::type
declare_integer_node_params(
  rclcpp::Node & node,
  const std::string & description,
  int64_t min = std::numeric_limits<int64_t>::min(),
  int64_t max = std::numeric_limits<int64_t>::max(),
  T default_value = 0)
{
  T output_value{default_value};
  try {
    auto param_desc = param_utils::int_param_description(description, min, max);
    output_value =
      static_cast<T>(node.declare_parameter<int64_t>(description, default_value, param_desc));
  } catch (const rclcpp::exceptions::InvalidParameterValueException & e) {
    std::ostringstream oss;
    oss << e.what();
    oss << " Parameter shall be in range [" << min << " , " << max << "]";
    throw rclcpp::exceptions::InvalidParameterValueException(oss.str());
  }
  return output_value;
}

rclcpp::Duration get_duration_from_node_param(
  rclcpp::Node & node,
  const std::string & node_name,
  int64_t default_value_sec = 0,
  int64_t default_value_nsec = 0)
{
  constexpr int64_t NANOSECONDS_PER_SECOND = 1000LL * 1000LL * 1000LL;
  constexpr int64_t MAX_NANOSEC = NANOSECONDS_PER_SECOND - 1;
  constexpr int64_t MIN_NANOSEC = -1 * MAX_NANOSEC;
  constexpr int64_t MAX_SEC = std::numeric_limits<int64_t>::max() / NANOSECONDS_PER_SECOND - 1;
  constexpr int64_t MIN_SEC = std::numeric_limits<int64_t>::min() / NANOSECONDS_PER_SECOND + 1;

  if (default_value_sec > MAX_SEC || default_value_sec < MIN_SEC) {
    std::ostringstream oss;
    oss << "default_value_sec = " << default_value_sec << " for " << node_name;
    oss << " is out of range. Shall be in range [" << MIN_SEC << " , " << MAX_SEC << "]";
    throw std::out_of_range(oss.str());
  }
  if (default_value_nsec > MAX_NANOSEC || default_value_nsec < MIN_NANOSEC) {
    std::ostringstream oss;
    oss << "default_value_nsec = " << default_value_nsec << " for " << node_name;
    oss << " is out of range. Shall be in range [" << MIN_NANOSEC << " , " << MAX_NANOSEC << "]";
    throw std::out_of_range(oss.str());
  }
  const auto sec = declare_integer_node_params<int64_t>(
    node, node_name + ".sec", MIN_SEC, MAX_SEC, default_value_sec);
  const auto nsec = declare_integer_node_params<int64_t>(
    node, node_name + ".nsec", MIN_NANOSEC, MAX_NANOSEC, default_value_nsec);

  auto total_nanoseconds =
    static_cast<int64_t>(std::abs(sec)) * NANOSECONDS_PER_SECOND + std::abs(nsec);
  if (sec < 0 || nsec < 0) {
    total_nanoseconds *= -1;
  }
  return rclcpp::Duration::from_nanoseconds(total_nanoseconds);
}

}  // namespace param_utils

PlayOptions get_play_options_from_node_params(rclcpp::Node & node)
{
  PlayOptions play_options{};
  play_options.read_ahead_queue_size = param_utils::declare_integer_node_params<size_t>(
    node, "read_ahead_queue_size", 1, std::numeric_limits<int64_t>::max(), 1000);

  play_options.node_prefix = node.declare_parameter<std::string>("node_prefix", "");

  auto desc_rate = param_utils::float_param_description(
    "Playback rate (hz)",
    0.000001,
    std::numeric_limits<float>::max());
  play_options.rate = static_cast<float>(node.declare_parameter<float>("rate", 1.0, desc_rate));

  play_options.topics_to_filter = node.declare_parameter<std::vector<std::string>>(
    "topics_to_filter", std::vector<std::string>());

  play_options.topics_regex_to_filter =
    node.declare_parameter<std::string>("topics_regex_to_filter", "");

  play_options.topics_regex_to_exclude =
    node.declare_parameter<std::string>("topics_regex_to_exclude", "");

  std::string qos_profile_overrides_path =
    node.declare_parameter<std::string>("qos_profile_overrides_path", "");

  if (!qos_profile_overrides_path.empty()) {
    try {
      YAML::Node yaml_file = YAML::LoadFile(qos_profile_overrides_path);
      for (auto topic_qos : yaml_file) {
        play_options.topic_qos_profile_overrides
        .emplace(
          topic_qos.first.as<std::string>(),
          topic_qos.second.as<rosbag2_storage::Rosbag2QoS>());
      }
    } catch (const YAML::Exception & ex) {
      throw std::runtime_error(
              std::string("Exception on parsing QoS ovverrides file: ") + ex.what());
    }
  }

  play_options.loop = node.declare_parameter<bool>("loop", false);

  auto topic_remapping_options = node.declare_parameter<std::vector<std::string>>(
    "topic_remapping_options", std::vector<std::string>());

  if (!topic_remapping_options.empty()) {
    RCLCPP_WARN(
      node.get_logger(),
      "Remappings shall be applied through standard CLI/Launch options."
      "'topic_remapping_options' content will be ignored.");
  }

  play_options.clock_publish_frequency =
    node.declare_parameter<double>("clock_publish_frequency", 0.0);

  play_options.clock_publish_on_topic_publish =
    node.declare_parameter<bool>("clock_publish_on_topic_publish", false);

  play_options.clock_trigger_topics = node.declare_parameter<std::vector<std::string>>(
    "clock_trigger_topics", std::vector<std::string>());

  play_options.delay = param_utils::get_duration_from_node_param(node, "delay", 0, 0);

  play_options.playback_duration = param_utils::get_duration_from_node_param(
    node, "playback_duration", -1, 0);

  play_options.playback_until_timestamp = param_utils::get_duration_from_node_param(
    node, "playback_until_timestamp", 0, -1).nanoseconds();

  play_options.start_paused = node.declare_parameter<bool>("start_paused", false);

  play_options.start_offset = param_utils::get_duration_from_node_param(
    node, "start_offset", 0, 0).nanoseconds();

  play_options.disable_keyboard_controls =
    node.declare_parameter<bool>("disable_keyboard_controls", false);

  play_options.wait_acked_timeout = param_utils::get_duration_from_node_param(
    node, "wait_acked_timeout", 0, -1).nanoseconds();

  play_options.disable_loan_message =
    node.declare_parameter<bool>("disable_loan_message", false);

  return play_options;
}

RecordOptions get_record_options_from_node_params(rclcpp::Node & node)
{
  RecordOptions record_options{};
  record_options.all = node.declare_parameter<bool>("all", false);

  record_options.is_discovery_disabled =
    node.declare_parameter<bool>("is_discovery_disabled", false);

  record_options.topics = node.declare_parameter<std::vector<std::string>>(
    "topics", std::vector<std::string>());

  record_options.rmw_serialization_format =
    node.declare_parameter<std::string>("rmw_serialization_format", "");

  record_options.topic_polling_interval = param_utils::get_duration_from_node_param(
    node, "topic_polling_interval",
    0, 1000000).to_chrono<std::chrono::milliseconds>();

  record_options.regex = node.declare_parameter<std::string>("regex", "");
  record_options.exclude = node.declare_parameter<std::string>("exclude", "");
  record_options.node_prefix = node.declare_parameter<std::string>("node_prefix", "");
  record_options.compression_mode = node.declare_parameter<std::string>("compression_mode", "");
  record_options.compression_format = node.declare_parameter<std::string>("compression_format", "");

  record_options.compression_queue_size = param_utils::declare_integer_node_params<uint64_t>(
    node, "compression_queue_size", 0, std::numeric_limits<int64_t>::max(), 1);

  record_options.compression_threads = param_utils::declare_integer_node_params<uint64_t>(
    node, "compression_threads", 0, std::numeric_limits<int64_t>::max(),
    record_options.compression_threads);

  std::string qos_profile_overrides_path =
    node.declare_parameter<std::string>("qos_profile_overrides_path", "");

  if (!qos_profile_overrides_path.empty()) {
    try {
      YAML::Node yaml_file = YAML::LoadFile(qos_profile_overrides_path);
      for (auto topic_qos : yaml_file) {
        record_options.topic_qos_profile_overrides
        .emplace(
          topic_qos.first.as<std::string>(),
          topic_qos.second.as<rosbag2_storage::Rosbag2QoS>());
      }
    } catch (const YAML::Exception & ex) {
      throw std::runtime_error(
              std::string("Exception on parsing QoS ovverrides file: ") + ex.what());
    }
  }

  record_options.include_hidden_topics =
    node.declare_parameter<bool>("include_hidden_topics", false);

  record_options.include_unpublished_topics =
    node.declare_parameter<bool>("include_unpublished_topics", false);

  record_options.ignore_leaf_topics =
    node.declare_parameter<bool>("ignore_leaf_topics", false);

  record_options.start_paused = node.declare_parameter<bool>("start_paused", false);

  record_options.use_sim_time = node.get_parameter("use_sim_time").get_value<bool>();

  if (record_options.use_sim_time && record_options.is_discovery_disabled) {
    throw std::invalid_argument(
            "'use_sim_time' and 'is_discovery_disabled' both set, but are incompatible settings. "
            "The `/clock` topic needs to be discovered to record with sim time.");
  }
  return record_options;
}

rosbag2_storage::StorageOptions
get_storage_options_from_node_params(rclcpp::Node & node)
{
  rosbag2_storage::StorageOptions storage_options{};

  storage_options.uri = node.declare_parameter<std::string>("uri", "");

  storage_options.storage_id = node.declare_parameter<std::string>("storage_id", "");

  storage_options.storage_config_uri =
    node.declare_parameter<std::string>("storage_config_uri", "");

  storage_options.max_bagfile_size = param_utils::declare_integer_node_params<uint64_t>(
    node, "max_bagfile_size", 0,
    std::numeric_limits<int64_t>::max(), storage_options.max_bagfile_size);

  storage_options.max_bagfile_duration = param_utils::declare_integer_node_params<uint64_t>(
    node, "max_bagfile_duration", 0,
    std::numeric_limits<int64_t>::max(), storage_options.max_bagfile_duration);

  storage_options.max_cache_size = param_utils::declare_integer_node_params<uint64_t>(
    node, "max_cache_size", 0,
    std::numeric_limits<int64_t>::max(), 100 * 1024 * 1024);

  storage_options.storage_preset_profile =
    node.declare_parameter<std::string>("storage_preset_profile", "");

  storage_options.snapshot_mode = node.declare_parameter<bool>("snapshot_mode", false);

  auto list_of_key_value_strings = node.declare_parameter<std::vector<std::string>>(
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

  storage_options.start_time_ns = param_utils::declare_integer_node_params<int64_t>(
    node, "start_time_ns", std::numeric_limits<int64_t>::min(),
    std::numeric_limits<int64_t>::max(), storage_options.start_time_ns);

  storage_options.end_time_ns = param_utils::declare_integer_node_params<int64_t>(
    node, "end_time_ns", std::numeric_limits<int64_t>::min(),
    std::numeric_limits<int64_t>::max(), storage_options.end_time_ns);

  return storage_options;
}

}  // namespace rosbag2_transport
