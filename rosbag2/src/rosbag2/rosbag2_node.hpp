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

#ifndef ROSBAG2__ROSBAG2_NODE_HPP_
#define ROSBAG2__ROSBAG2_NODE_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/node.hpp"
#include "rcutils/types.h"

#include "generic_publisher.hpp"
#include "generic_subscription.hpp"

namespace rosbag2
{

class Rosbag2Node : public rclcpp::Node
{
public:
  explicit Rosbag2Node(const std::string & node_name);
  ~Rosbag2Node() override = default;

  std::shared_ptr<GenericPublisher> create_generic_publisher(
    const std::string & topic, const std::string & type);

  std::shared_ptr<GenericSubscription> create_generic_subscription(
    const std::string & topic,
    const std::string & type,
    std::function<void(std::shared_ptr<rcutils_char_array_t>)> callback);

  std::map<std::string, std::string>
  get_topics_with_types(const std::vector<std::string> & topic_names);

  std::map<std::string, std::string> get_all_topics_with_types();

  std::map<std::string, std::string> sanitize_topics_and_types(
    std::map<std::string, std::vector<std::string>> topics_and_types);
};

}  // namespace rosbag2

#endif  // ROSBAG2__ROSBAG2_NODE_HPP_
