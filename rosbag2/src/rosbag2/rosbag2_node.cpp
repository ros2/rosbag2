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

#include "typesupport_helpers.hpp"

namespace rosbag2
{

Rosbag2Node::Rosbag2Node(const std::string & node_name)
: rclcpp::Node(node_name)
{}

std::shared_ptr<GenericPublisher> Rosbag2Node::create_generic_publisher(
  const std::string & topic, const std::string & type)
{
  auto type_support = get_typesupport(type);
  return std::make_shared<GenericPublisher>(get_node_base_interface().get(), topic, *type_support);
}

std::shared_ptr<GenericSubscription> Rosbag2Node::create_generic_subscription(
  const std::string & topic,
  const std::string & type,
  std::function<void(std::shared_ptr<rcutils_char_array_t>)> callback)
{
  auto type_support = get_typesupport(type);

  auto subscription = std::make_shared<GenericSubscription>(
    get_node_base_interface()->get_shared_rcl_node_handle(),
    *type_support,
    topic,
    callback);

  get_node_topics_interface()->add_subscription(subscription, nullptr);

  return subscription;
}

}  // namespace rosbag2
