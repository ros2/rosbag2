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

#ifndef ROSBAG2_TEST_COMMON__MEMORY_MANAGEMENT_HPP_
#define ROSBAG2_TEST_COMMON__MEMORY_MANAGEMENT_HPP_

#include <memory>
#include <string>

#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"

namespace rosbag2_test_common
{
class MemoryManagement
{
public:
  MemoryManagement() = default;

  ~MemoryManagement() = default;

  template<typename T>
  inline
  std::shared_ptr<rmw_serialized_message_t>
  serialize_message(std::shared_ptr<T> message)
  {
    rclcpp::Serialization<T> ser;
    rclcpp::SerializedMessage serialized_message;
    ser.serialize_message(message.get(), &serialized_message);

    auto ret = std::make_shared<rmw_serialized_message_t>();
    *ret = serialized_message.release_rcl_serialized_message();
    return ret;
  }

  template<typename T>
  inline
  std::shared_ptr<T>
  deserialize_message(std::shared_ptr<rmw_serialized_message_t> serialized_message)
  {
    rclcpp::Serialization<T> ser;
    auto message = std::make_shared<T>();
    rclcpp::SerializedMessage rclcpp_serialized_message(*serialized_message);
    ser.deserialize_message(&rclcpp_serialized_message, message.get());
    return message;
  }

  std::shared_ptr<rmw_serialized_message_t> make_initialized_message()
  {
    rclcpp::SerializedMessage serialized_message(0u);
    auto ret = std::make_shared<rmw_serialized_message_t>();
    *ret = serialized_message.release_rcl_serialized_message();
    return ret;
  }
};

}  // namespace rosbag2_test_common

#endif  // ROSBAG2_TEST_COMMON__MEMORY_MANAGEMENT_HPP_
