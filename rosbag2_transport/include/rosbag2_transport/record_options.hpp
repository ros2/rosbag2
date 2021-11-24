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
  bool all = false;
  bool is_discovery_disabled = false;
  std::vector<std::string> topics;
  std::string rmw_serialization_format;
  std::chrono::milliseconds topic_polling_interval{100};
  std::string regex = "";
  std::string exclude = "";
  std::string node_prefix = "";
  std::string compression_mode = "";
  std::string compression_format = "";
  uint64_t compression_queue_size = 1;
  uint64_t compression_threads = 0;
  std::unordered_map<std::string, rclcpp::QoS> topic_qos_profile_overrides{};
  bool include_hidden_topics = false;
  bool start_paused = false;
};

}  // namespace rosbag2_transport

namespace YAML
{
template<>
struct ROSBAG2_TRANSPORT_PUBLIC convert<rosbag2_transport::RecordOptions>
{
  static Node encode(const rosbag2_transport::RecordOptions & storage_options);
  static bool decode(const Node & node, rosbag2_transport::RecordOptions & storage_options);
};
}  // namespace YAML

#endif  // ROSBAG2_TRANSPORT__RECORD_OPTIONS_HPP_
