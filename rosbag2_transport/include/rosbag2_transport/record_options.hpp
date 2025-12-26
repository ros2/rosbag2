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

#ifndef ROSBAG2_TRANSPORT__RECORD_OPTIONS_HPP_
#define ROSBAG2_TRANSPORT__RECORD_OPTIONS_HPP_

#include <chrono>
#include <string>
#include <unordered_map>
#include <vector>

#include "keyboard_handler/keyboard_handler.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosbag2_storage/yaml.hpp"
#include "rosbag2_transport/visibility_control.hpp"

namespace rosbag2_transport
{
struct RecordOptions
{
public:
  bool all_topics = false;
  bool all_services = false;
  bool all_actions = false;
  bool is_discovery_disabled = false;
  std::vector<std::string> topics;
  std::vector<std::string> topic_types;
  std::vector<std::string> services;  // service event topics list
  std::vector<std::string> actions;   // actions name list
  std::vector<std::string> exclude_topics;
  std::vector<std::string> exclude_topic_types;
  std::vector<std::string> exclude_service_events;  // service event topics list
  std::vector<std::string> exclude_actions;         // actions name list
  // rmw_serialization_format deprecated. Use output_serialization_format instead
  std::string rmw_serialization_format;
  std::string input_serialization_format;
  std::string output_serialization_format;
  std::chrono::milliseconds topic_polling_interval{100};
  std::string regex = "";
  std::string exclude_regex = "";
  std::string node_prefix = "";
  /// \brief Compression mode. Valid values are "file", "message" or "" (no compression).
  /// \Note: To use compression mode "message", the underlying storage must support this parameter.
  /// For mcap storage, need to use specific compression options provided via
  /// StorageOptions::storage_config_uri.
  std::string compression_mode = "";
  /// \brief Compression format. E.g., "zstd", "bz2", "lz4", etc. Shall be used together with
  /// compression_mode parameter.
  std::string compression_format = "";
  uint64_t compression_queue_size = 0;
  /// \brief // The number of compression threads
  uint64_t compression_threads = 0;
  /// \brief Compression threads scheduling priority.
  /// For Windows the valid values are: THREAD_PRIORITY_LOWEST=-2, THREAD_PRIORITY_BELOW_NORMAL=-1
  /// and THREAD_PRIORITY_NORMAL=0. For POSIX compatible OSes this is the "nice" value.
  /// The nice value range is -20 to +19 where -20 is highest, 0 default and +19 is lowest.
  int32_t compression_threads_priority = 0;
  /// \brief Path to a YAML file containing topic QoS profile overrides.
  /// The YAML file must map topic names to rclcpp::QoS profiles.
  std::unordered_map<std::string, rclcpp::QoS> topic_qos_profile_overrides{};
  /// \brief Include hidden topics. i.e., starting any token of the name with an underscore.
  bool include_hidden_topics = false;
  /// \brief Include unpublished topics. i.e., topics that have no publishers (only subscribers).
  bool include_unpublished_topics = false;
  /// \brief Ignore leaf topics. i.e., topics that have no subscribers.
  bool ignore_leaf_topics = false;
  /// \brief Start recording in paused state if true.
  /// \Note This parameter is only used during construction of the Recorder class. To pause
  /// or resume an ongoing recording use the pause and resume services or direct public API.
  bool start_paused = false;
  /// \brief Use simulated time (if true).
  bool use_sim_time = false;
  /// \brief Disable keyboard controls if true. This parameter is only used during construction of
  /// the Recorder class to decide whether to initialize KeyboardHandler class or not.
  bool disable_keyboard_controls = false;
  /// \brief Maximum rate in times per second (Hz) at which the statistics about lost messages
  /// will be published. If set to 0, no statistics will be published. The value must be greater
  /// than or equal to 0 and less than or equal to 1000.
  float statistics_max_publishing_rate = 1.0f;
};

}  // namespace rosbag2_transport

namespace YAML
{
template<>
struct ROSBAG2_TRANSPORT_PUBLIC convert<rosbag2_transport::RecordOptions>
{
  static Node encode(const rosbag2_transport::RecordOptions & storage_options);
  static bool decode(
    const Node & node, rosbag2_transport::RecordOptions & storage_options, int version = 9);
};
}  // namespace YAML

#endif  // ROSBAG2_TRANSPORT__RECORD_OPTIONS_HPP_
