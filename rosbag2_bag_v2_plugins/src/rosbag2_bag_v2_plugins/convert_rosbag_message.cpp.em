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

#include "convert_rosbag_message.hpp"

#include <cstdint>
#include <memory>
#include <string>

#include "ros1_bridge/bridge.hpp"
#include "ros1_bridge/factory_interface.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rcutils/allocator.h"
#include "rosbag2_storage/ros_helper.hpp"
#include "rosbag2/typesupport_helpers.hpp"
#include "rosbag2/types/introspection_message.hpp"

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

void
convert_1_to_2(
  const std::string & ros1_type_name,
  ros::serialization::IStream & ros1_message_stream,
  std::shared_ptr<rosbag2_introspection_message_t> ros2_message)
{
  @[if not mappings]@
  (void) ros1_type_name;
  (void) ros1_message_stream;
  (void) ros2_message;
  @[end if]@

  std::string ros2_type_name;
  ros1_bridge::get_1to2_mapping(ros1_type_name, ros2_type_name);

  @[for m in mappings]@
  if (ros1_type_name == "@(m.ros1_msg.package_name)/@(m.ros1_msg.message_name)"
    && ros2_type_name == "@(m.ros2_msg.package_name)/@(m.ros2_msg.message_name)")
  {
    @(m.ros1_msg.package_name)::@(m.ros1_msg.message_name) typed_ros1_message;

    ros::serialization
      ::Serializer<@(m.ros1_msg.package_name)::@(m.ros1_msg.message_name)>
      ::read(ros1_message_stream, typed_ros1_message);

    auto factory = ros1_bridge::get_factory(
      "@(m.ros1_msg.package_name)/@(m.ros1_msg.message_name)",
      "@(m.ros2_msg.package_name)/@(m.ros2_msg.message_name)");
    factory->convert_1_to_2(&typed_ros1_message, ros2_message->message);
  }
  @[end for]@
}
}  // end namespace rosbag2_bag_v2_plugin