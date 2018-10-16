// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#ifndef ROSBAG2_STORAGE__SERIALIZED_BAG_MESSAGE_HPP_
#define ROSBAG2_STORAGE__SERIALIZED_BAG_MESSAGE_HPP_

#include <memory>
#include <string>

#include "rcutils/types/char_array.h"
#include "rcutils/time.h"

extern "C"
{

typedef struct rosbag2_storage_serialized_bag_message_t
{
  rcutils_char_array_t * serialized_data;
  rcutils_time_point_value_t time_stamp;
  rcutils_char_array_t topic_name;
  rcutils_allocator_t allocator;
} rosbag2_storage_serialized_bag_message_t;

rosbag2_storage_serialized_bag_message_t rosbag2_get_zero_initialized_serialized_bag_message();

RCUTILS_WARN_UNUSED
int rosbag2_serialized_bag_message_init(
  rosbag2_storage_serialized_bag_message_t * message,
  rcutils_time_point_value_t time_stamp,
  char * topic_name,
  const rcutils_allocator_t * allocator);

RCUTILS_WARN_UNUSED
int rosbag2_serialized_bag_message_fini(rosbag2_storage_serialized_bag_message_t * message);

};

#endif  // ROSBAG2_STORAGE__SERIALIZED_BAG_MESSAGE_HPP_
