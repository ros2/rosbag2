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
#include "rcl/graph.h"

#include "rcutils/logging_macros.h"

#include "std_msgs/msg/string.hpp"

#include "rosbag2_storage/serialized_bag_message.hpp"
#include "rosbag2_storage/storage_interfaces/read_only_interface.hpp"
#include "rosbag2_storage/storage_interfaces/read_write_interface.hpp"
#include "rosbag2_storage/storage_factory.hpp"
#include "raw_subscription.hpp"
#include "rosbag2_node.hpp"
#include "typesupport_helpers.hpp"

namespace rosbag2
{

const char * ROS_PACKAGE_NAME = "rosbag2";

void Rosbag2::record(
  const std::string & file_name,
  const std::string & topic_name,
  std::function<void(void)> after_write_action)
{
  (void) after_write_action;
  rosbag2_storage::StorageFactory factory;
  auto storage = factory.open_read_write(file_name, "sqlite3");

  if (storage) {
    RCUTILS_LOG_INFO_NAMED(ROS_PACKAGE_NAME, "Waiting for messages...");

    auto node = std::make_shared<rclcpp::Node>("rosbag2");

    bool topic_found = false;
    std::string type;
    while (!topic_found && rclcpp::ok()) {
      auto topics = node->get_topic_names_and_types();
      // TODO(Martin_Idel-SI): check topics
      std::string complete_topic = "/" + topic_name;
      auto position = topics.find(complete_topic);
      if (position != topics.end()) {
        topic_found = true;
        // TODO(Martin_Idel-SI): handle case of multiple types
        type = position->second[0];
        break;
      }
    }

    auto type_support = get_typesupport(type);
    auto subscription = std::make_shared<RawSubscription>(
      node->get_node_base_interface()->get_shared_rcl_node_handle(),
      *type_support,
      topic_name,
      [storage, topic_name, after_write_action](std::shared_ptr<rcutils_char_array_t> message) {
        auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
        bag_message->serialized_data = message;
        bag_message->topic_name = topic_name;
        rcutils_time_point_value_t time_stamp;
        int error = rcutils_system_time_now(&time_stamp);
        if (error != RCUTILS_RET_OK) {
          RCUTILS_LOG_ERROR_NAMED(
            "rosbag2", "Error getting current time. Error: %s", rcutils_get_error_string_safe());
        }
        bag_message->time_stamp = time_stamp;

        storage->write(bag_message);
        if (after_write_action) {
          after_write_action();
        }
      });

    node->get_node_topics_interface()->add_subscription(subscription, nullptr);

    storage->create_topic(topic_name, type);
    while (rclcpp::ok()) {
      rclcpp::spin(node);
    }
  }
}

void Rosbag2::play(const std::string & file_name, const std::string & topic_name)
{
  rosbag2_storage::StorageFactory factory;
  auto storage = factory.open_read_only(file_name, "sqlite3");

  if (storage) {
    std::string type = storage->read_topic_type(topic_name);
    auto type_support = get_typesupport(type);

    auto node = std::make_shared<Rosbag2Node>("rosbag2_node");
    auto publisher = node->create_raw_publisher(topic_name, *type_support);

    while (storage->has_next()) {
      auto message = storage->read_next();
      // without the sleep_for() many messages are lost.
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      publisher->publish(message);
      RCUTILS_LOG_INFO_NAMED(ROS_PACKAGE_NAME, "published message");
    }
  }
}

}  // namespace rosbag2
