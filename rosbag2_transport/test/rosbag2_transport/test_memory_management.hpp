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

#ifndef ROSBAG2_TRANSPORT__TEST_MEMORY_MANAGEMENT_HPP_
#define ROSBAG2_TRANSPORT__TEST_MEMORY_MANAGEMENT_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"

namespace test_helpers
{
class TestMemoryManagement
{
public:
  TestMemoryManagement();
  ~TestMemoryManagement() = default;

  std::shared_ptr<rmw_serialized_message_t> get_initialized_serialized_message(size_t capacity);

  template<typename T>
  inline
  const rosidl_message_type_support_t * get_message_typesupport(std::shared_ptr<T>)
  {
    return rosidl_typesupport_cpp::get_message_type_support_handle<T>();
  }

  template<typename T>
  inline
  std::shared_ptr<rmw_serialized_message_t> serialize_message(std::shared_ptr<T> message)
  {
    auto serialized_message = get_initialized_serialized_message(0);
    auto error = rmw_serialize(
      message.get(),
      get_message_typesupport(message),
      serialized_message.get());
    if (error != RCL_RET_OK) {
      throw std::runtime_error("Failed to serialize");
    }
    return serialized_message;
  }

  template<typename T>
  inline
  std::shared_ptr<T> deserialize_message(std::shared_ptr<rmw_serialized_message_t> serialized_msg)
  {
    auto message = std::make_shared<T>();
    auto error = rmw_deserialize(
      serialized_msg.get(),
      get_message_typesupport(message),
      message.get());
    if (error != RCL_RET_OK) {
      throw std::runtime_error("Failed to deserialize");
    }
    return message;
  }

private:
  rcutils_allocator_t rcutils_allocator_;
};


}  // namespace test_helpers

#endif  // ROSBAG2_TRANSPORT__TEST_MEMORY_MANAGEMENT_HPP_
