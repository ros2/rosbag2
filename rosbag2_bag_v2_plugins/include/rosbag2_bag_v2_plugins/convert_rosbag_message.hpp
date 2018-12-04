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

#ifndef ROSBAG2_BAG_V2_PLUGINS__CONVERT_ROSBAG_MESSAGE_HPP_
#define ROSBAG2_BAG_V2_PLUGINS__CONVERT_ROSBAG_MESSAGE_HPP_

#include <memory>
#include <string>

#include "rosbag/message_instance.h"
#include "rosbag2/types.hpp"
#include "rosbag2/types/introspection_message.hpp"

namespace rosbag2_bag_v2_plugins
{
bool get_1to2_mapping(const std::string & ros1_message_type, std::string & ros2_message_type);

void
convert_1_to_2(
  const std::string & ros1_type_name,
  ros::serialization::IStream & ros1_message_stream,
  std::shared_ptr<rosbag2_introspection_message_t> ros2_message);
}  // namespace rosbag2_bag_v2_plugins

#endif  // ROSBAG2_BAG_V2_PLUGINS__CONVERT_ROSBAG_MESSAGE_HPP_
