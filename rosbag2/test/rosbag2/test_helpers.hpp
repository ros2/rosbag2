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

#ifndef ROSBAG2__TEST_HELPERS_HPP_
#define ROSBAG2__TEST_HELPERS_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "../../src/rosbag2/typesupport_helpers.hpp"

namespace test_helpers
{

std::shared_ptr<rcutils_char_array_t> serialize_string_message(std::string message)
{
  auto test_message = std::make_shared<std_msgs::msg::String>();
  test_message->data = message;

  auto rcutils_allocator = rcutils_get_default_allocator();
  auto initial_capacity = 8u + static_cast<size_t>(test_message->data.size());
  auto msg = new rcutils_char_array_t;
  *msg = rcutils_get_zero_initialized_char_array();
  auto ret = rcutils_char_array_init(msg, initial_capacity, &rcutils_allocator);
  if (ret != RCUTILS_RET_OK) {
    throw std::runtime_error("Error allocating resources for serialized message" +
            std::to_string(ret));
  }

  auto serialized_message = std::shared_ptr<rcutils_char_array_t>(msg,
      [](rcutils_char_array_t * msg) {
        int error = rcutils_char_array_fini(msg);
        delete msg;
        if (error != RCUTILS_RET_OK) {
          RCUTILS_LOG_ERROR_NAMED(
            "rosbag2", "Leaking memory. Error: %s", rcutils_get_error_string_safe());
        }
      });

  serialized_message->buffer_length = initial_capacity;

  auto string_ts = rosbag2::get_typesupport("std_msgs/String");

  auto error = rmw_serialize(test_message.get(), string_ts, serialized_message.get());
  if (error != RMW_RET_OK) {
    throw std::runtime_error("Something went wrong preparing the serialized message");
  }

  return serialized_message;
}

std::string deserialize_string_message(std::shared_ptr<rcutils_char_array_t> serialized_message)
{
  char * copied = new char[serialized_message->buffer_length];
  auto string_length = serialized_message->buffer_length - 8;
  memcpy(copied, &serialized_message->buffer[8], string_length);
  std::string message_content(copied);
  delete[] copied;
  return message_content;
}


}  // namespace test_helpers

#endif  // ROSBAG2__TEST_HELPERS_HPP_
