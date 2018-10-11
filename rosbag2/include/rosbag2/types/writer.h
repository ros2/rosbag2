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

#ifndef ROSBAG2__WRITER_H_
#define ROSBAG2__WRITER_H_

#include "rosbag2/types/bag_handle.h"

extern "C"
{
// internal struct with c++ data members
// contains std::unique_ptr<rosbag2_storage::storage_interfaces::ReadWriteInterface> storage;
// contains std::unique_ptr<rosbag2_storage::Rosbag2StorageFactory> factory;
struct WriterImpl;

typedef struct rosbag2_writer_t
{
  WriterImpl * impl;
} rosbag2_writer_t;

rosbag2_writer_t get_zero_initalized_writer();

RCUTILS_WARN_UNUSED
int rosbag2_writer_init(rosbag2_bag_handle_t * bag_handle);

RCUTILS_WARN_UNUSED
int rosbag2_writer_fini(rosbag2_writer_t * writer);

int rosbag2_create_topic(
  rosbag2_writer_t * writer, rcutils_char_array_t * topic, rcutils_string_array_t * type);

int rosbag2_write(
  rosbag2_writer_t * writer, rosbag2_serialized_bag_message_t * message);
};

#endif  // ROSBAG2__WRITER_H_
