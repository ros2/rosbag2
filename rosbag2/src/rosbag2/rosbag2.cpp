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

#include "rosbag2/rosbag2.hpp"

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "rcutils/logging_macros.h"

#include "std_msgs/msg/string.hpp"

#include "rosbag2_storage/serialized_bag_message.hpp"
#include "rosbag2_storage/storage_interfaces/read_only_interface.hpp"
#include "rosbag2_storage/storage_interfaces/read_write_interface.hpp"
#include "rosbag2_storage/storage_factory.hpp"

namespace rosbag2
{

const char * ROS_PACKAGE_NAME = "rosbag2";

void Rosbag2::record(
  const std::string & file_name,
  const std::string & topic_name,
  std::function<void(void)> after_write_action)
{
  rosbag2_storage::StorageFactory factory;
  auto storage = factory.open_read_write(file_name, "sqlite3");
  storage->create_topic();

  if (storage) {
    auto node = std::make_shared<rclcpp::Node>("rosbag_node");
    auto subscription = node->create_subscription<std_msgs::msg::String>(
      topic_name,
      [&storage, after_write_action](std::shared_ptr<rmw_serialized_message_t> msg) {
        auto bag_msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
        bag_msg->serialized_data = msg;
        rcutils_time_point_value_t time_stamp;
        int error = rcutils_system_time_now(&time_stamp);
        if (error != RCUTILS_RET_OK) {
          RCUTILS_LOG_ERROR_NAMED(
            "rosbag2", "Error getting current time. Error: %s", rcutils_get_error_string_safe());
        }
        bag_msg->time_stamp = time_stamp;
        storage->write(bag_msg);
        if (after_write_action) {
          after_write_action();
        }
      });

    RCUTILS_LOG_INFO_NAMED(ROS_PACKAGE_NAME, "Waiting for messages...");
    rclcpp::spin(node);
  }
}

void Rosbag2::play(const std::string & file_name, const std::string & topic_name)
{
  rosbag2_storage::StorageFactory factory;
  auto storage = factory.open_read_only(file_name, "sqlite3");

  if (storage) {
    auto node = std::make_shared<rclcpp::Node>("rosbag_publisher_node");
    auto publisher = node->create_publisher<std_msgs::msg::String>(topic_name);
    while (storage->has_next()) {
      auto message = storage->read_next();

      // without the sleep_for() many messages are lost.
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      publisher->publish(message->serialized_data.get());
      RCUTILS_LOG_INFO_NAMED(ROS_PACKAGE_NAME, "published message");
    }
  }
}

}  // namespace rosbag2
