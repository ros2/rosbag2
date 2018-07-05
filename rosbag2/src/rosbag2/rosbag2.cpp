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

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "storage/sqlite/sqlite_storage.hpp"

using namespace rosbag2;

int main(int argc, const char** argv)
{
  SqliteStorage storage;
  storage.open("test.db");

  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("rosbag_node");
  auto subscription = node->create_subscription<std_msgs::msg::String>(
    "string_topic",
    [&storage](std_msgs::msg::String::ConstSharedPtr msg) {
      std::cout << msg->data << std::endl;
      storage.insertMessage(msg->data);
  });

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}



