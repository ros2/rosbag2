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

#ifndef ROSBAG2__ROSBAG2_HPP_
#define ROSBAG2__ROSBAG2_HPP_

#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rosbag2/visibility_control.hpp"

namespace rosbag2
{

class Rosbag2
{
public:
  ROSBAG2_PUBLIC
  void record(
    const std::string & file_name,
    const std::string & topic_name,
    std::function<void(void)> after_write_action = nullptr);

  ROSBAG2_PUBLIC
  void play(const std::string & file_name, const std::string & topic_name);

  ROSBAG2_PUBLIC
  std::string wait_for_topic(
    const std::string & topic_name, const std::shared_ptr<rclcpp::Node> & node);
};

}  // namespace rosbag2

#endif  // ROSBAG2__ROSBAG2_HPP_
