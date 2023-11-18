// Copyright 2023 Patrick Roncagliolo.
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
}  // namespace param_utils


PlayOptions get_play_options_from_node_params(rclcpp::Node * node)
{
  PlayOptions play_options{};
  auto desc_raqs = param_utils::int_param_description(
    "Read ahead queue size (messages)",
    1,
    std::numeric_limits<int64_t>::max());
  auto read_ahead_queue_size_ = node->declare_parameter<int64_t>(
    "read_ahead_queue_size",
    1000,
    desc_raqs);
  play_options.read_ahead_queue_size = static_cast<uint64_t>(read_ahead_queue_size_);

  play_options.node_prefix = node->declare_parameter<std::string>(
    "node_prefix",
    "");

  auto desc_rate = param_utils::float_param_description(
    "Playback rate (hz)",
    0.000001,
    std::numeric_limits<float>::max());
  play_options.rate = node->declare_parameter<float>(
    "rate",
    1.0,
    desc_rate);

  play_options.topics_to_filter = node->declare_parameter<std::vector<std::string>>(
    "topics_to_filter",
    std::vector<std::string>());

  play_options.topics_regex_to_filter = node->declare_parameter<std::string>(
    "topics_regex_to_filter",
    "");

  play_options.topics_regex_to_exclude = node->declare_parameter<std::string>(
    "topics_regex_to_exclude",
    "");

  std::string qos_profile_overrides_path = node->declare_parameter<std::string>(
    "qos_profile_overrides_path", ""
  );

  if (qos_profile_overrides_path != "") {
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
              std::string("Exception on parsing QoS ovverrides file: ") +
              ex.what());
    }
  }

  play_options.loop = node->declare_parameter<bool>(
    "loop",
    false);

  auto topic_remapping_options = node->declare_parameter<std::vector<std::string>>(
    "topic_remapping_options",
    std::vector<std::string>());

  if (!topic_remapping_options.empty()) {
    RCLCPP_WARN(
      node->get_logger(),
      "Remappings shall be applied through standard CLI/Launch options."
      "'topic_remapping_options' content will be ignored.");
  }

  play_options.clock_publish_frequency = node->declare_parameter<double>(
    "clock_publish_frequency",
    0.0);

  play_options.clock_publish_on_topic_publish = node->declare_parameter<bool>(
    "clock_publish_on_topic_publish",
    false);

  play_options.clock_trigger_topics = node->declare_parameter<std::vector<std::string>>(
    "clock_trigger_topics",
    std::vector<std::string>());

  auto delay_sec = node->declare_parameter<int32_t>(
    "delay.sec",
    0.0);
  auto delay_nsec = node->declare_parameter<int32_t>(
    "delay.nsec",
    0.0);
  play_options.delay = rclcpp::Duration(delay_sec, delay_nsec);

  auto playback_duration_sec = node->declare_parameter<int32_t>(
    "playback_duration.sec",
    -1.0);
  auto playback_duration_nsec = node->declare_parameter<int32_t>(
    "playback_duration.nsec",
    0.0);
  play_options.playback_duration = rclcpp::Duration(playback_duration_sec, playback_duration_nsec);

  auto playback_until_timestamp_sec = node->declare_parameter<int32_t>(
    "playback_until_timestamp.sec",
    0.0);
  auto playback_until_timestamp_nsec = node->declare_parameter<int32_t>(
    "playback_until_timestamp.nsec",
    0.0);
  play_options.playback_until_timestamp = rclcpp::Duration(
    playback_until_timestamp_sec,
    playback_until_timestamp_nsec).
    nanoseconds();

  play_options.start_paused = node->declare_parameter<bool>(
    "start_paused",
    false);

  auto start_offset_sec = node->declare_parameter<int32_t>(
    "start_offset.sec",
    0.0);
  auto start_offset_nsec = node->declare_parameter<int32_t>(
    "start_offset.nsec",
    0.0);
  play_options.start_offset = rclcpp::Duration(start_offset_sec, start_offset_nsec).nanoseconds();

  play_options.disable_keyboard_controls = node->declare_parameter<bool>(
    "disable_keyboard_controls",
    false);

  auto wait_acked_timeout_sec = node->declare_parameter<int32_t>(
    "wait_acked_timeout.sec",
    -1.0);
  auto wait_acked_timeout_nsec = node->declare_parameter<int32_t>(
    "wait_acked_timeout.nsec",
    0.0);
  play_options.wait_acked_timeout =
    rclcpp::Duration(wait_acked_timeout_sec, wait_acked_timeout_nsec).nanoseconds();

  play_options.disable_loan_message = node->declare_parameter<bool>(
    "disable_loan_message",
    false);

  return play_options;
}

RecordOptions get_record_options_from_node_params(rclcpp::Node * node)
{
  RecordOptions record_options{};
  record_options.all = node->declare_parameter<bool>(
    "all",
    false);

  record_options.is_discovery_disabled = node->declare_parameter<bool>(
    "is_discovery_disabled",
    false);

  record_options.topics = node->declare_parameter<std::vector<std::string>>(
    "topics",
    std::vector<std::string>());

  record_options.rmw_serialization_format = node->declare_parameter<std::string>(
    "rmw_serialization_format",
    "");

  auto topic_polling_interval_sec = node->declare_parameter<int32_t>(
    "topic_polling_interval.sec",
    0.0);
  auto topic_polling_interval_nsec = node->declare_parameter<int32_t>(
    "topic_polling_interval.nsec",
    1000000.0);
  record_options.topic_polling_interval = rclcpp::Duration(
    topic_polling_interval_sec,
    topic_polling_interval_nsec).to_chrono<std::chrono::milliseconds>();

  record_options.regex = node->declare_parameter<std::string>(
    "regex",
    "");

  record_options.exclude = node->declare_parameter<std::string>(
    "exclude",
    "");

  record_options.node_prefix = node->declare_parameter<std::string>(
    "node_prefix",
    "");

  record_options.compression_mode = node->declare_parameter<std::string>(
    "compression_mode",
    "");

  record_options.compression_format = node->declare_parameter<std::string>(
    "compression_format",
    "");

  auto desc_cqs = param_utils::int_param_description(
    "Compression queue size (messages)",
    1,
    std::numeric_limits<int64_t>::max());
  auto compression_queue_size_ = node->declare_parameter<int64_t>(
    "compression_queue_size",
    1,
    desc_cqs);
  record_options.compression_queue_size = static_cast<uint64_t>(compression_queue_size_);

  auto desc_cts = param_utils::int_param_description(
    "Compression threads",
    0,
    std::numeric_limits<int64_t>::max());
  auto compression_threads_ = node->declare_parameter<int64_t>(
    "compression_threads",
    0,
    desc_cts);
  record_options.compression_threads = static_cast<uint64_t>(compression_threads_);

  std::string qos_profile_overrides_path = node->declare_parameter<std::string>(
    "qos_profile_overrides_path", ""
  );

  if (qos_profile_overrides_path != "") {
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
              std::string("Exception on parsing QoS ovverrides file: ") +
              ex.what());
    }
  }

  record_options.include_hidden_topics = node->declare_parameter<bool>(
    "include_hidden_topics",
    false);

  record_options.include_unpublished_topics = node->declare_parameter<bool>(
    "include_unpublished_topics",
    false);

  record_options.ignore_leaf_topics = node->declare_parameter<bool>(
    "ignore_leaf_topics",
    false);

  record_options.start_paused = node->declare_parameter<bool>(
    "start_paused",
    false);

  record_options.use_sim_time = node->get_parameter("use_sim_time").get_value<bool>();


  if (record_options.use_sim_time && record_options.is_discovery_disabled) {
    throw std::invalid_argument(
            "'use_sim_time' and 'is_discovery_disabled' both set, but are incompatible settings. "
            "The `/clock` topic needs to be discovered to record with sim time.");
  }
  return record_options;
}

rosbag2_storage::StorageOptions
get_storage_options_from_node_params(rclcpp::Node * node)
{
  rosbag2_storage::StorageOptions storage_options{};

  storage_options.uri = node->declare_parameter<std::string>("uri", "");

  storage_options.storage_id = node->declare_parameter<std::string>("storage_id", "");

  storage_options.storage_config_uri =
    node->declare_parameter<std::string>("storage_config_uri", "");

  auto desc_mbs = param_utils::int_param_description(
    "Max bagfile size (bytes)",
    0,
    std::numeric_limits<int64_t>::max());
  auto max_bagfile_size_ = node->declare_parameter<int64_t>(
    "max_bagfile_size",
    0,
    desc_mbs);
  storage_options.max_bagfile_size = static_cast<uint64_t>(max_bagfile_size_);

  auto desc_mbd = param_utils::int_param_description(
    "Max bagfile duration (nanoseconds)",
    0,
    std::numeric_limits<int64_t>::max());
  auto max_bagfile_duration_ = node->declare_parameter<int64_t>(
    "max_bagfile_duration",
    0,
    desc_mbd);
  storage_options.max_bagfile_duration = static_cast<uint64_t>(max_bagfile_duration_);

  auto desc_mcs = param_utils::int_param_description(
    "Max chache size (messages)",
    0,
    std::numeric_limits<int64_t>::max());
  auto max_cache_size_ = node->declare_parameter<int64_t>(
    "max_cache_size",
    0,
    desc_mcs);
  storage_options.max_cache_size = static_cast<uint64_t>(max_cache_size_);

  storage_options.storage_preset_profile =
    node->declare_parameter<std::string>("storage_preset_profile", "");

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

  storage_options.start_time_ns = node->declare_parameter<int64_t>(
    "start_time_ns",
    -1);
  storage_options.end_time_ns = node->declare_parameter<int64_t>(
    "end_time_ns",
    -1);

  return storage_options;
}

}  // namespace rosbag2_transport
