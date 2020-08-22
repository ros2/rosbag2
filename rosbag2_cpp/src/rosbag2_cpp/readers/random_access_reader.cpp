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

std::shared_ptr<rosbag2_storage::SerializedBagMessage> RandomAccessReader::read_at_index(int index)
{
  if (storage_) {
    auto message = storage_->read_at_index(index);
    return converter_ ? converter_->convert(message) : message;
  }
  throw std::runtime_error("Bag is not open. Call open() before reading.");
}



}  // namespace readers
}  // namespace rosbag2_cpp
