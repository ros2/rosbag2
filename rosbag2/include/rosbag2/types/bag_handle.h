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

#ifndef ROSBAG2__TYPES__BAG_HANDLE_H_
#define ROSBAG2__TYPES__BAG_HANDLE_H_

#include "rcutils/types.h"
#include "rosbag2/visibility_control.hpp"

extern "C"
{

typedef struct rosbag2_bag_handle_t
{
  rcutils_char_array_t uri;
  rcutils_char_array_t storage_identifier;
  rcutils_char_array_t storage_format;
  rcutils_allocator_t allocator;
} rosbag2_bag_handle_t;

rosbag2_bag_handle_t get_zero_initialized_bag_handle();

RCUTILS_WARN_UNUSED
ROSBAG2_PUBLIC
int rosbag2_bag_handle_init(
  rosbag2_bag_handle_t * bag_handle,
  char * uri,
  char * storage_identifier,
  char * storage_format,
  const rcutils_allocator_t * allocator);

RCUTILS_WARN_UNUSED
ROSBAG2_PUBLIC
int rosbag2_bag_handle_fini(rosbag2_bag_handle_t * bag_handle);

};

#endif // ROSBAG2__TYPES__BAG_HANDLE_H_
