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

#ifndef ROSBAG2__SEQUENTIAL_READER_H_
#define ROSBAG2__SEQUENTIAL_READER_H_

#include "rosbag2/types/bag_handle.h"
#include "rosbag2/types/serialized_bag_message.h"

extern "C"
{

// internal struct with c++ data members
// contains std::unique_ptr<rosbag2_storage::storage_interfaces::ReadOnlyInterface> storage;
// contains std::unique_ptr<rosbag2_storage::Rosbag2StorageFactory> factory;
struct SequentialReaderImpl;

typedef struct rosbag2_sequential_reader_t
{
  SequentialReaderImpl * impl;
} rosbag2_sequential_reader_t;

rosbag2_sequential_reader_t get_zero_initalized_sequential_reader();

RCUTILS_WARN_UNUSED
int rosbag2_sequential_reader_init(rosbag2_bag_handle_t * bag_handle);

RCUTILS_WARN_UNUSED
int rosbag2_sequential_reader_fini(rosbag2_sequential_reader_t * reader);

bool rosbag2_has_next(rosbag2_sequential_reader_t * reader);

int rosbag2_read_next(
  rosbag2_sequential_reader_t * reader, rosbag2_serialized_bag_message_t * message);

int rosbag2_read_all_topics_and_types(
  rosbag2_sequential_reader_t * reader, rmw_names_and_types_t * topics_and_types);
};

#endif  // ROSBAG2__SEQUENTIAL_READER_H_
