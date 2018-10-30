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

#ifndef ROSBAG2_CONVERTER_DEFAULT_PLUGINS__CDR__CDR_CONVERTER_HPP_
#define ROSBAG2_CONVERTER_DEFAULT_PLUGINS__CDR__CDR_CONVERTER_HPP_

#include <memory>
#include <string>

#include "rosbag2/serialization_format_converter_interface.hpp"

namespace rosbag2_converter_default_plugins
{

class CdrConverter : public rosbag2::SerializationFormatConverterInterface
{
public:
  void deserialize(
    std::shared_ptr<const rosbag2::SerializedBagMessage> serialized_message,
    const rosidl_message_type_support_t * type_support,
    std::shared_ptr<rosbag2_ros2_message_t> ros_message) override;

  void serialize(
    std::shared_ptr<const rosbag2_ros2_message_t> ros_message,
    const rosidl_message_type_support_t * type_support,
    std::shared_ptr<rosbag2::SerializedBagMessage> serialized_message) override;
};

}  // namespace rosbag2_converter_default_plugins

#endif  // ROSBAG2_CONVERTER_DEFAULT_PLUGINS__CDR__CDR_CONVERTER_HPP_
