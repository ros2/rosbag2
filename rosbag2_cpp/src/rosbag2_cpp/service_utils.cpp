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

#include "rcl/service_introspection.h"

#include "rosbag2_cpp/service_utils.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"

#include "service_msgs/msg/service_event_info.hpp"

namespace rosbag2_cpp
{
const char * service_event_topic_type_postfix = "_Event";
const char * service_event_topic_type_middle = "/srv/";

bool is_service_event_topic(const std::string & topic, const std::string & topic_type)
{
  if (topic.length() <= strlen(RCL_SERVICE_INTROSPECTION_TOPIC_POSTFIX)) {
    return false;
  }

  std::string end_topic_name = topic.substr(
    topic.length() - strlen(RCL_SERVICE_INTROSPECTION_TOPIC_POSTFIX));

  // Should be "/_service_event"
  if (end_topic_name != RCL_SERVICE_INTROSPECTION_TOPIC_POSTFIX) {
    return false;
  }

  if (topic_type.length() <= std::strlen(service_event_topic_type_postfix)) {
    return false;
  }

  // Should include '/srv/' in type
  if (topic_type.find(service_event_topic_type_middle) == std::string::npos) {
    return false;
  }

  if (topic_type.length() <= std::strlen(service_event_topic_type_postfix)) {
    return false;
  }

  return topic_type.compare(
    topic_type.length() - std::strlen(service_event_topic_type_postfix),
    std::strlen(service_event_topic_type_postfix),
    service_event_topic_type_postfix) == 0;
}

std::string service_event_topic_name_to_service_name(const std::string & topic_name)
{
  std::string service_name;
  if (topic_name.length() <= strlen(RCL_SERVICE_INTROSPECTION_TOPIC_POSTFIX)) {
    return service_name;
  }

  if (topic_name.substr(
      topic_name.length() -
      strlen(RCL_SERVICE_INTROSPECTION_TOPIC_POSTFIX)) !=
    RCL_SERVICE_INTROSPECTION_TOPIC_POSTFIX)
  {
    return service_name;
  }

  service_name = topic_name.substr(
    0, topic_name.length() - strlen(RCL_SERVICE_INTROSPECTION_TOPIC_POSTFIX));

  return service_name;
}

std::string service_event_topic_type_to_service_type(const std::string & topic_type)
{
  std::string service_type;
  if (topic_type.length() <= std::strlen(service_event_topic_type_postfix)) {
    return service_type;
  }

  // Should include '/srv/' in type
  if (topic_type.find(service_event_topic_type_middle) == std::string::npos) {
    return service_type;
  }

  if (topic_type.substr(topic_type.length() - std::strlen(service_event_topic_type_postfix)) !=
    service_event_topic_type_postfix)
  {
    return service_type;
  }

  service_type = topic_type.substr(
    0, topic_type.length() - strlen(service_event_topic_type_postfix));

  return service_type;
}

size_t get_serialization_size_for_service_metadata_event()
{
  // Since the size is fixed, it only needs to be calculated once.
  static size_t size = 0;

  if (size != 0) {
    return size;
  }

  const rosidl_message_type_support_t * type_support_info =
    rosidl_typesupport_cpp::get_message_type_support_handle<service_msgs::msg::ServiceEventInfo>();

  // Get the serialized size of service event info
  const rosidl_message_type_support_t * type_support_handle =
    rosidl_typesupport_cpp::get_message_typesupport_handle_function(
    type_support_info,
    rosidl_typesupport_introspection_cpp::typesupport_identifier);
  if (type_support_handle == nullptr) {
    throw std::runtime_error("Cannot get ServiceEventInfo typesupport handle !");
  }

  auto service_event_info =
    static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(
    type_support_handle->data);

  // endian type (4 size) + service event info size + empty request (4 bytes)
  // + emtpy response (4 bytes)
  size = 4 + service_event_info->size_of_ + 4 + 4;

  return size;
}

std::string service_name_to_service_event_topic_name(const std::string & service_name)
{
  if (service_name.empty()) {
    return service_name;
  }

  // If the end of string is RCL_SERVICE_INTROSPECTION_TOPIC_POSTFIX, do nothing
  if ((service_name.length() > strlen(RCL_SERVICE_INTROSPECTION_TOPIC_POSTFIX)) &&
    (service_name.substr(service_name.length() - strlen(RCL_SERVICE_INTROSPECTION_TOPIC_POSTFIX)) ==
    RCL_SERVICE_INTROSPECTION_TOPIC_POSTFIX))
  {
    return service_name;
  }

  return service_name + RCL_SERVICE_INTROSPECTION_TOPIC_POSTFIX;
}

}  // namespace rosbag2_cpp
