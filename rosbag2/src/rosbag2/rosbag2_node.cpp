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

#include "rosbag2_node.hpp"

#include <memory>
#include <string>

namespace rosbag2
{

Rosbag2Node::Rosbag2Node(const std::string & node_name)
: rclcpp::Node(node_name)
{}

std::shared_ptr<RawPublisher> Rosbag2Node::create_raw_publisher(
  const std::string & topic, const rosidl_message_type_support_t & type_support)
{
  return std::make_shared<RawPublisher>(get_node_base_interface().get(), topic, type_support);
}

}  // namespace rosbag2
