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

#include "rosbag_v_2_deserializer_plugin.hpp"

#include <memory>

#include "rosbag/message_instance.h"

#include "rosbag2_bag_v2_plugins/storage/convert_rosbag_message.hpp"
#include "rosbag2/types/introspection_message.hpp"

namespace rosbag2_bag_v2_plugins
{
void RosbagV2DeserializerPlugin::deserialize(
  std::shared_ptr<const rosbag2::SerializedBagMessage> serialized_message,
  const rosidl_message_type_support_t * type_support,
  std::shared_ptr<rosbag2_introspection_message_t> ros_message)
{
  (void) type_support;

  convert_1_to_2(
    reinterpret_cast<rosbag::MessageInstance *>(serialized_message->serialized_data->buffer),
    ros_message);

  ros_message->time_stamp = serialized_message->time_stamp;
  rosbag2::introspection_message_set_topic_name(
    ros_message.get(), serialized_message->topic_name.c_str());
}
}  // namespace rosbag2_bag_v2_plugins
