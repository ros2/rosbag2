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

#ifndef ROSBAG2__SERIALIZATION_FORMAT_CONVERTER_INTERFACE_HPP_
#define ROSBAG2__SERIALIZATION_FORMAT_CONVERTER_INTERFACE_HPP_

#include <memory>
#include <string>

#include "rosbag2/types/ros2_message.hpp"
#include "rosbag2/types.hpp"
#include "rcutils/types.h"
#include "rosbag2_storage/serialized_bag_message.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"

namespace rosbag2
{

class SerializationFormatConverterInterface
{
public:
  virtual ~SerializationFormatConverterInterface() = default;

  virtual void deserialize(
    std::shared_ptr<rosbag2_ros2_message_t> ros_message,
    std::shared_ptr<const SerializedBagMessage> serialized_message,
    const rosidl_message_type_support_t * type_support) = 0;

  virtual void serialize(
    std::shared_ptr<SerializedBagMessage> serialized_message,
    std::shared_ptr<const rosbag2_ros2_message_t> ros_message,
    const rosidl_message_type_support_t * type_support) = 0;
};

}  // namespace rosbag2

#endif  // ROSBAG2__SERIALIZATION_FORMAT_CONVERTER_INTERFACE_HPP_
