/*
 *  Copyright (c) 2018,  Bosch Software Innovations GmbH.
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */
#include "rosbag2/rosbag2.hpp"

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "storage/sqlite/sqlite_storage.hpp"

namespace rosbag2
{

void record(const std::string & file_name, const std::string & topic_name)
{
  SqliteStorage storage(file_name);

  if (storage.create()) {
    auto node = std::make_shared<rclcpp::Node>("rosbag_node");
    auto subscription = node->create_subscription<std_msgs::msg::String>(
      topic_name,
      [&storage](std_msgs::msg::String::ConstSharedPtr msg) {
        storage.write(msg->data);
      });

    std::cout << "Waiting for messages..." << std::endl;
    rclcpp::spin(node);
  }
}

}  // namespace rosbag2
