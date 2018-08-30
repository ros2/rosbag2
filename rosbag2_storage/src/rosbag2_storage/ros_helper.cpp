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

#include "rosbag2_storage/ros_helper.hpp"

#include <memory>
#include <string>

#include "rcutils/logging_macros.h"
#include "rcutils/types.h"

namespace rosbag2_storage
{

std::shared_ptr<rcutils_char_array_t>
make_serialized_message(const void * data, size_t size)
{
  auto rcutils_allocator = new rcutils_allocator_t;
  *rcutils_allocator = rcutils_get_default_allocator();
  auto msg = new rcutils_char_array_t;
  *msg = rcutils_get_zero_initialized_char_array();
  auto ret = rcutils_char_array_init(msg, size, rcutils_allocator);
  if (ret != RCUTILS_RET_OK) {
    throw std::runtime_error("Error allocating resources for serialized message: " +
            std::string(rcutils_get_error_string_safe()));
  }

  auto serialized_message = std::shared_ptr<rcutils_char_array_t>(msg,
      [](rcutils_char_array_t * msg) {
        int error = rcutils_char_array_fini(msg);
        delete msg;
        if (error != RCUTILS_RET_OK) {
          RCUTILS_LOG_ERROR_NAMED(
            "rosbag2_storage",
            "Leaking memory. Error: %s", rcutils_get_error_string_safe());
        }
      });

  memcpy(serialized_message->buffer, data, size);
  serialized_message->buffer_length = size;

  return serialized_message;
}

}  // namespace rosbag2_storage
