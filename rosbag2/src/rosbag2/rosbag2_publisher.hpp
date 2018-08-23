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

#ifndef ROSBAG2__ROSBAG2_PUBLISHER_HPP_
#define ROSBAG2__ROSBAG2_PUBLISHER_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "rosbag2_storage/serialized_bag_message.hpp"

namespace rosbag2
{

class Rosbag2Publisher : public rclcpp::PublisherBase
{
public:
  Rosbag2Publisher(
    rclcpp::node_interfaces::NodeBaseInterface * node_base,
    const std::string & topic,
    const rosidl_message_type_support_t & type_support);

  ~Rosbag2Publisher() = default;

  void publish(std::shared_ptr<rosbag2_storage::SerializedBagMessage> message);
};

}  // namespace rosbag2

#endif  // ROSBAG2__ROSBAG2_PUBLISHER_HPP_
