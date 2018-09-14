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

#include "test_memory_management.hpp"

#include <memory>
#include <string>

#include "rosbag2_transport/logging.hpp"

namespace test_helpers
{

TestMemoryManagement::TestMemoryManagement()
{
  rcutils_allocator_ = rcutils_get_default_allocator();
}

std::shared_ptr<rmw_serialized_message_t>
TestMemoryManagement::get_initialized_serialized_message(size_t capacity)
{
  auto msg = new rmw_serialized_message_t;
  *msg = rcutils_get_zero_initialized_char_array();
  auto ret = rcutils_char_array_init(msg, capacity, &rcutils_allocator_);
  if (ret != RCUTILS_RET_OK) {
    throw std::runtime_error("Error allocating resources for serialized message: " +
            std::string(rcutils_get_error_string_safe()));
  }

  auto serialized_message = std::shared_ptr<rmw_serialized_message_t>(msg,
      [](rmw_serialized_message_t * msg) {
        int error = rcutils_char_array_fini(msg);
        delete msg;
        if (error != RCUTILS_RET_OK) {
          ROSBAG2_TRANSPORT_LOG_ERROR("Leaking memory. Error: %s", rcutils_get_error_string_safe());
        }
      });
  return serialized_message;
}


}  // namespace test_helpers
