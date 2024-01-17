// Copyright 2023 Sony Group Corporation.
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

#ifndef ROSBAG2_CPP__SERVICE_UTILS_HPP_
#define ROSBAG2_CPP__SERVICE_UTILS_HPP_

#include <array>
#include <string>

#include "rosbag2_cpp/visibility_control.hpp"

#include "service_msgs/msg/service_event_info.hpp"

namespace rosbag2_cpp
{
ROSBAG2_CPP_PUBLIC
bool
is_service_event_topic(const std::string & topic, const std::string & topic_type);

// Call this function after is_service_event_topic() return true
ROSBAG2_CPP_PUBLIC
std::string
service_event_topic_name_to_service_name(const std::string & topic_name);

// Call this function after is_service_event_topic() return true
ROSBAG2_CPP_PUBLIC
std::string
service_event_topic_type_to_service_type(const std::string & topic_type);

ROSBAG2_CPP_PUBLIC
size_t
get_serialization_size_for_service_metadata_event();

ROSBAG2_CPP_PUBLIC
std::string
service_name_to_service_event_topic_name(const std::string & service_name);

ROSBAG2_CPP_PUBLIC
bool
introspection_include_metadata_and_contents(size_t message_size);

ROSBAG2_CPP_PUBLIC
std::string
client_id_to_string(std::array<uint8_t, 16> & client_id);

struct client_id_hash
{
  static_assert(
    std::is_same<std::array<uint8_t, 16>,
    service_msgs::msg::ServiceEventInfo::_client_gid_type>::value);
  std::size_t operator()(const std::array<uint8_t, 16> & client_id) const
  {
    std::hash<uint8_t> hasher;
    std::size_t seed = 0;
    for (const auto & value : client_id) {
      // 0x9e3779b9 is from https://cryptography.fandom.com/wiki/Tiny_Encryption_Algorithm
      seed ^= hasher(value) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};
}  // namespace rosbag2_cpp

#endif  // ROSBAG2_CPP__SERVICE_UTILS_HPP_
