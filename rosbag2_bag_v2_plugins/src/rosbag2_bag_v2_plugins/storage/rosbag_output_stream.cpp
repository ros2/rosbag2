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

#include "rosbag_output_stream.hpp"

#include <cstdint>
#include <memory>
#include <string>

#include "rosbag2_storage/ros_helper.hpp"

namespace rosbag2_bag_v2_plugins
{

RosbagOutputStream::RosbagOutputStream(const std::string & type)
{
  char_array_ = rosbag2_storage::make_serialized_message(type.c_str(), type.length() + 1);
}

uint8_t * RosbagOutputStream::advance(size_t size)
{
  auto old_capacity = char_array_->buffer_capacity;
  auto ret = rcutils_uint8_array_resize(char_array_.get(), size + old_capacity);
  if (ret != RCUTILS_RET_OK) {
    throw std::runtime_error("No memory available. Error code " + std::to_string(ret));
  }
  char_array_->buffer_length = char_array_->buffer_capacity;
  return char_array_->buffer + old_capacity;
}

std::shared_ptr<rcutils_uint8_array_t> RosbagOutputStream::get_content()
{
  return char_array_;
}

}  // namespace rosbag2_bag_v2_plugins
