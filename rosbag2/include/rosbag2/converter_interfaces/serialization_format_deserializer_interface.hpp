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

#ifndef ROSBAG2__CONVERTER_INTERFACES__SERIALIZATION_FORMAT_DESERIALIZER_INTERFACE_HPP_
#define ROSBAG2__CONVERTER_INTERFACES__SERIALIZATION_FORMAT_DESERIALIZER_INTERFACE_HPP_

#include <memory>

#include "rosbag2/types/introspection_message.hpp"
#include "rosbag2/types.hpp"

namespace rosbag2
{

class SerializationFormatDeserializerInterface
{
public:
  virtual ~SerializationFormatDeserializerInterface() = default;

  virtual void deserialize(
    std::shared_ptr<const rosbag2::SerializedBagMessage> serialized_message,
    const rosidl_message_type_support_t * type_support,
    std::shared_ptr<rosbag2_introspection_message_t> ros_message) = 0;
};

}  // namespace rosbag2

#endif  // ROSBAG2__CONVERTER_INTERFACES__SERIALIZATION_FORMAT_DESERIALIZER_INTERFACE_HPP_
