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

#ifndef ROSBAG2__TYPES__INTROSPECTION_MESSAGE_HPP_
#define ROSBAG2__TYPES__INTROSPECTION_MESSAGE_HPP_

#include <memory>

#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rcutils/time.h"
#include "rcutils/allocator.h"
#include "rosbag2/visibility_control.hpp"

typedef struct rosbag2_introspection_message_t
{
  void * message;
  char * topic_name;
  rcutils_time_point_value_t time_stamp;
  rcutils_allocator_t allocator;
} rosbag2_ros2_message_t;

namespace rosbag2
{

ROSBAG2_PUBLIC
std::shared_ptr<rosbag2_introspection_message_t>
allocate_introspection_message(
  const rosidl_message_type_support_t * introspection_ts, const rcutils_allocator_t * allocator);

ROSBAG2_PUBLIC
void introspection_message_set_topic_name(
  rosbag2_introspection_message_t * msg, const char * topic_name);

ROSBAG2_PUBLIC
void allocate_internal_types(
  void * msg, const rosidl_typesupport_introspection_cpp::MessageMembers * members);

ROSBAG2_PUBLIC
void deallocate_introspection_message(
  rosbag2_introspection_message_t * msg,
  const rosidl_message_type_support_t * introspection_ts);

}  // namespace rosbag2

#endif  // ROSBAG2__TYPES__INTROSPECTION_MESSAGE_HPP_
