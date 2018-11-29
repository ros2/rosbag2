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
@{
from ros1_bridge import camel_case_to_lower_case_underscore
}@

#include "rosbag2_bag_v2_plugins/storage/convert_rosbag_message.hpp"

#include <string>

#include "ros1_bridge/bridge.hpp"
#include "ros1_bridge/factory_interface.hpp"

#include "rosbag2_storage/ros_helper.hpp"
#include "rosbag2/typesupport_helpers.hpp"
#include "rclcpp/rclcpp.hpp"

@[for m in mappings]@
#include "@(m.ros1_msg.package_name)/@(m.ros1_msg.message_name).h"
#include "@(m.ros2_msg.package_name)/msg/@(camel_case_to_lower_case_underscore(m.ros2_msg.message_name)).hpp"
@[end for]@

namespace rosbag2_bag_v2_plugins
{

bool get_1to2_mapping(const std::string & ros1_message_type, std::string & ros2_message_type)
{
  return ros1_bridge::get_1to2_mapping(ros1_message_type, ros2_message_type);
}

std::shared_ptr<rosbag2_storage::SerializedBagMessage>
convert_1_to_2(rosbag::MessageInstance msg_instance)
{
  @[if not mappings]@
  (void)msg_instance;
  @[end if]@

  std::string ros1_type_name = msg_instance.getDataType();
  std::string ros2_type_name;
  ros1_bridge::get_1to2_mapping(ros1_type_name, ros2_type_name);

  @[for m in mappings]@
  if (ros1_type_name == "@(m.ros1_msg.package_name)/@(m.ros1_msg.message_name)"
    && ros2_type_name == "@(m.ros2_msg.package_name)/@(m.ros2_msg.message_name)")
  {
    auto ros1_message =
      msg_instance.instantiate<@(m.ros1_msg.package_name)::@(m.ros1_msg.message_name)>();
    @(m.ros2_msg.package_name)::msg::@(m.ros2_msg.message_name) ros2_message;
    auto factory = ros1_bridge::get_factory(
      "@(m.ros1_msg.package_name)/@(m.ros1_msg.message_name)",
      "@(m.ros2_msg.package_name)/@(m.ros2_msg.message_name)");
    factory->convert_1_to_2(ros1_message.get(), &ros2_message);

    auto serialized_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
    serialized_message->serialized_data = rosbag2_storage::make_empty_serialized_message(0);
    auto typesupport = rosbag2::get_typesupport(
      "@(m.ros2_msg.package_name)/@(m.ros2_msg.message_name)", "rosidl_typesupport_cpp");
    serialized_message->topic_name = msg_instance.getTopic();
    serialized_message->time_stamp = msg_instance.getTime().toNSec();

    auto ret =
      rmw_serialize(&ros2_message, typesupport, serialized_message->serialized_data.get());

    return ret == RMW_RET_OK ? serialized_message : nullptr;
  }
  @[end for]@

  return std::shared_ptr<rosbag2_storage::SerializedBagMessage>();
}
}  // end namespace rosbag2_bag_v2_plugin