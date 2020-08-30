// Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rosbag2_cpp/readers/random_access_reader.hpp"


namespace rosbag2_cpp
{
namespace readers
{

std::shared_ptr<rosbag2_storage::SerializedBagMessage> RandomAccessReader::read_at_timestamp(rcutils_time_point_value_t timestamp)
{
  if (storage_) {
    auto message = storage_->read_at_timestamp(timestamp);
    return converter_ ? converter_->convert(message) : message;
  }
  throw std::runtime_error("Bag is not open. Call open() before reading.");
}

std::shared_ptr<rosbag2_storage::SerializedBagMessage> RandomAccessReader::read_at_index(uint32_t index)
{
  if (storage_) {
    auto message = storage_->read_at_index(index);
    return converter_ ? converter_->convert(message) : message;
  }
  throw std::runtime_error("Bag is not open. Call open() before reading.");
}

std::shared_ptr<std::vector<rosbag2_storage::SerializedBagMessage>> RandomAccessReader::read_at_timestamp_range(rcutils_time_point_value_t timestamp_begin, rcutils_time_point_value_t timestamp_end)
{
  if (storage_) {
    auto message_vector = storage_->read_at_timestamp_range(timestamp_begin, timestamp_end);
    if (converter_) {
      for (auto &message : *message_vector) {
        message = *(converter_->convert(std::make_shared<rosbag2_storage::SerializedBagMessage>(message)));
      }
    }
    return message_vector;
  } else {
    throw std::runtime_error("Bag is not open. Call open() before reading.");
  }
}
std::shared_ptr<std::vector<rosbag2_storage::SerializedBagMessage>> RandomAccessReader::read_at_index_range(uint32_t index_begin, uint32_t index_end)
{
  if (storage_) {
    auto message_vector = storage_->read_at_index_range(index_begin, index_end);
    if (converter_) {
      for (auto &message : *message_vector) {
        message = *(converter_->convert(std::make_shared<rosbag2_storage::SerializedBagMessage>(message)));
      }
    }
    return message_vector;
  } else {
    throw std::runtime_error("Bag is not open. Call open() before reading.");
  }
}

}  // namespace readers
}  // namespace rosbag2_cpp
