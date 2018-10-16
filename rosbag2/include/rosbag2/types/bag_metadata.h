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

#ifndef ROSBAG2__TYPES__BAG_METADATA_H_
#define ROSBAG2__TYPES__BAG_METADATA_H_

#include "rcutils/strdup.h"

#include "rosbag2_storage/bag_metadata.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"
#include "rosbag2_storage/topic_with_type.hpp"
#include "rosbag2/visibility_control.hpp"

extern "C" {

typedef struct rosbag2_topic_metadata_t
{
  size_t message_count;
  rcutils_char_array_t topic_name;
  rcutils_char_array_t topic_type;
} rosbag2_topic_metadata_t;

typedef struct rosbag2_topic_metadata_array_t
{
  size_t size;
  rosbag2_topic_metadata_t * data;
  rcutils_allocator_t allocator;
} rosbag2_topic_metadata_array_t;

typedef struct rosbag2_bag_metadata_t
{
  int version = 1;
  size_t bag_size;
  rcutils_char_array_t storage_identifier;
  rcutils_char_array_t storage_format;
  rcutils_string_array_t relative_file_paths;
  rcutils_time_point_value_t duration;
  rcutils_time_point_value_t starting_time;
  size_t message_count;
  rosbag2_topic_metadata_array_t topics_with_message_count;
} rosbag2_bag_metadata_t;

ROSBAG2_PUBLIC
rosbag2_bag_metadata_t get_zero_initialized_bag_metadata();

ROSBAG2_PUBLIC
RCUTILS_WARN_UNUSED
int rosbag2_bag_metadata_init(
  rosbag2_bag_metadata_t * bag_metadata, const rcutils_allocator_t * allocator);

ROSBAG2_PUBLIC
RCUTILS_WARN_UNUSED
int rosbag2_bag_metadata_fini(rosbag2_bag_metadata_t * bag_metadata);

ROSBAG2_PUBLIC
rosbag2_topic_metadata_t get_zero_initialized_topic_metadata();

ROSBAG2_PUBLIC
RCUTILS_WARN_UNUSED
int rosbag2_topic_metadata_init(
  rosbag2_topic_metadata_t * metadata, const rcutils_allocator_t * allocator);

ROSBAG2_PUBLIC
RCUTILS_WARN_UNUSED
int rosbag2_topic_metadata_fini(rosbag2_topic_metadata_t * topic_metadata);

rosbag2_topic_metadata_array_t get_zero_initialized_topic_metadata_array();

ROSBAG2_PUBLIC
RCUTILS_WARN_UNUSED
int rosbag2_topic_metadata_array_init(
  rosbag2_topic_metadata_array_t * topic_metadata_array,
  size_t size,
  const rcutils_allocator_t * allocator);

ROSBAG2_PUBLIC
RCUTILS_WARN_UNUSED
int rosbag2_topic_metadata_array_fini(rosbag2_topic_metadata_array_t * topic_metadata_array);

};

#endif  // ROSBAG2__TYPES_BAG_METADATA_H_
