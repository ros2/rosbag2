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

#include <cstring>
#include <string>

#include "rcl/service_introspection.h"

#include "rosbag2_cpp/service_utils.hpp"
#include "service_msgs/msg/service_event_info.hpp"

namespace rosbag2_cpp
{
const char * kServiceEventTopicTypePostfix = "_Event";
const char * kServiceEventTopicTypeMiddle = "/srv/";
const size_t kServiceEventTopicPostfixLen = strlen(RCL_SERVICE_INTROSPECTION_TOPIC_POSTFIX);
const size_t kServiceEventTypePostfixLen = strlen(kServiceEventTopicTypePostfix);

bool is_service_event_topic(const std::string & topic_name, const std::string & topic_type)
{
  if (topic_name.length() <= kServiceEventTopicPostfixLen) {
    return false;
  } else {
    // The end of the topic name should be "/_service_event"
    if (topic_name.substr(topic_name.length() - kServiceEventTopicPostfixLen) !=
      RCL_SERVICE_INTROSPECTION_TOPIC_POSTFIX)
    {
      return false;
    }
  }

  if (topic_type.length() <= kServiceEventTypePostfixLen) {
    return false;
  } else {
    // Should include '/srv/' in type
    if (topic_type.find(kServiceEventTopicTypeMiddle) == std::string::npos) {
      return false;
    }

    return topic_type.compare(
      topic_type.length() - kServiceEventTypePostfixLen,
      kServiceEventTypePostfixLen,
      kServiceEventTopicTypePostfix) == 0;
  }
}

std::string service_event_topic_name_to_service_name(const std::string & topic_name)
{
  std::string service_name;
  if (topic_name.length() <= kServiceEventTopicPostfixLen) {
    return service_name;
  } else {
    // The end of the topic name should be "/_service_event"
    if (topic_name.substr(topic_name.length() - kServiceEventTopicPostfixLen) ==
      RCL_SERVICE_INTROSPECTION_TOPIC_POSTFIX)
    {
      service_name = topic_name.substr(0, topic_name.length() - kServiceEventTopicPostfixLen);
    }
    return service_name;
  }
}

std::string service_event_topic_type_to_service_type(const std::string & topic_type)
{
  std::string service_type;
  if (topic_type.length() <= kServiceEventTypePostfixLen) {
    return service_type;
  }
  // Should include '/srv/' in type
  if (topic_type.find(kServiceEventTopicTypeMiddle) == std::string::npos) {
    return service_type;
  }

  if (topic_type.substr(topic_type.length() - kServiceEventTypePostfixLen) !=
    kServiceEventTopicTypePostfix)
  {
    return service_type;
  }
  service_type = topic_type.substr(0, topic_type.length() - kServiceEventTypePostfixLen);

  return service_type;
}

std::string service_name_to_service_event_topic_name(const std::string & service_name)
{
  if (service_name.empty()) {
    return service_name;
  }

  // If the end of string is RCL_SERVICE_INTROSPECTION_TOPIC_POSTFIX, do nothing
  if ((service_name.length() > kServiceEventTopicPostfixLen) &&
    (service_name.substr(service_name.length() - kServiceEventTopicPostfixLen) ==
    RCL_SERVICE_INTROSPECTION_TOPIC_POSTFIX))
  {
    return service_name;
  }
  return service_name + RCL_SERVICE_INTROSPECTION_TOPIC_POSTFIX;
}

std::string client_id_to_string(std::array<uint8_t, 16> & client_id)
{
  // output format:
  // xx-xx-xx-xx-xx-xx-xx-xx-xx-xx-xx-xx-xx-xx-xx-xx
  std::string client_id_string = std::to_string(client_id[0]);
  for (int i = 1; i < 16; i++) {
    client_id_string += "-" + std::to_string(client_id[i]);
  }
  return client_id_string;
}

std::size_t client_id_hash::operator()(const std::array<uint8_t, 16> & client_id) const
{
  std::hash<uint8_t> hasher;
  std::size_t seed = 0;
  for (const auto & value : client_id) {
    // 0x9e3779b9 is from https://cryptography.fandom.com/wiki/Tiny_Encryption_Algorithm
    seed ^= hasher(value) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  }
  return seed;
}
}  // namespace rosbag2_cpp
