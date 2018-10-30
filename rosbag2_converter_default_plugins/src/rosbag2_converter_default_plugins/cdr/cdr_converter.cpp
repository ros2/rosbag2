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

#include "cdr_converter.hpp"

#include <memory>
#include <string>

#include "rcutils/strdup.h"
#include "rmw/rmw.h"
#include "../logging.hpp"

namespace rosbag2_converter_default_plugins
{

void CdrConverter::deserialize(
  const std::shared_ptr<const rosbag2::SerializedBagMessage> serialized_message,
  const rosidl_message_type_support_t * type_support,
  std::shared_ptr<rosbag2_ros2_message_t> ros_message)
{
  ros_message->topic_name = serialized_message->topic_name.c_str();
  ros_message->timestamp = serialized_message->time_stamp;

  auto ret =
    rmw_deserialize(serialized_message->serialized_data.get(), type_support, ros_message->message);
  if (ret != RMW_RET_OK) {
    ROSBAG2_CONVERTER_DEFAULT_PLUGINS_LOG_ERROR("Failed to deserialize message.");
  }
}

void CdrConverter::serialize(
  const std::shared_ptr<const rosbag2_ros2_message_t> ros_message,
  const rosidl_message_type_support_t * type_support,
  std::shared_ptr<rosbag2::SerializedBagMessage> serialized_message)
{
  serialized_message->topic_name = std::string(ros_message->topic_name);
  serialized_message->time_stamp = ros_message->timestamp;

  auto ret = rmw_serialize(
    ros_message->message, type_support, serialized_message->serialized_data.get());
  if (ret != RMW_RET_OK) {
    ROSBAG2_CONVERTER_DEFAULT_PLUGINS_LOG_ERROR("Failed to serialize message.");
  }
}

}  // namespace rosbag2_converter_default_plugins

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(rosbag2_converter_default_plugins::CdrConverter,
  rosbag2::SerializationFormatConverterInterface)
