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

#ifndef UTILS__SERIALIZED_MESSAGE_UTILS_HPP_
#define UTILS__SERIALIZED_MESSAGE_UTILS_HPP_

#include <memory>
#include <stdexcept>
#include <string>

#include "rcutils/snprintf.h"

#include "rmw/types.h"

namespace utils
{

rmw_serialized_message_t create_binary_string_message()
{
  auto serialized_msg = rmw_get_zero_initialized_serialized_message();
  auto allocator = rcutils_get_default_allocator();
  auto initial_capacity = 100ul;
  auto ret = rmw_serialized_message_init(
    &serialized_msg,
    initial_capacity,
    &allocator);
  if (ret != RCL_RET_OK) {
    throw std::runtime_error("failed to initialize serialized message");
  }

  auto copied_bytes = rcutils_snprintf(
    serialized_msg.buffer, initial_capacity, "%c%c%c%c%c%c%c%c%s",
    0x00, 0x01, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00, "Hello World!");
  serialized_msg.buffer_length = copied_bytes + 1;  // +1 for null terminator

  return serialized_msg;
}

void print_serialized_message(rmw_serialized_message_t * serialized_msg)
{
  fprintf(stderr, "payload size: %zu\n", serialized_msg->buffer_length);
  fprintf(stderr, "capcity size: %zu\n", serialized_msg->buffer_capacity);
  for (auto i = 0u; i < serialized_msg->buffer_length; ++i) {
    fprintf(stderr, "%02x ", serialized_msg->buffer[i]);
  }
  fprintf(stderr, "\n");
}

void destroy_binary_string_message(rmw_serialized_message_t * serialized_msg)
{
  auto ret = rmw_serialized_message_fini(serialized_msg);
  if (ret != RCL_RET_OK) {
    throw std::runtime_error("failed to cleanup serialized message");
  }
}

}  // namespace utils

#endif  // UTILS__SERIALIZED_MESSAGE_UTILS_HPP_
