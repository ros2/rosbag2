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

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcl/graph.h"

#include "rcutils/logging_macros.h"

#include "rosbag2_storage/serialized_bag_message.hpp"
#include "rosbag2_storage/storage_factory.hpp"
#include "rosbag2_storage/storage_interfaces/read_only_interface.hpp"
#include "rosbag2_storage/storage_interfaces/read_write_interface.hpp"

#include "std_msgs/msg/string.hpp"

#include "generic_subscription.hpp"
#include "rosbag2_node.hpp"
#include "typesupport_helpers.hpp"

namespace rosbag2
{

const char * ROS_PACKAGE_NAME = "rosbag2";

std::string Rosbag2::get_topic_type(
  const std::string & topic_name, const std::shared_ptr<rclcpp::Node> & node)
{
  // TODO(Martin-Idel-SI): This is a short sleep to allow the node some time to discover the topic
  // This should be replaced by an auto-discovery system in the future
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  auto topics = node->get_topic_names_and_types();
  std::string complete_topic_name = topic_name;
  if (topic_name[0] != '/') {
    complete_topic_name = "/" + topic_name;
  }
  auto position = topics.find(complete_topic_name);
  if (position != topics.end()) {
    if (position->second.size() > 1) {
      RCUTILS_LOG_ERROR_NAMED(
        ROS_PACKAGE_NAME,
        "Topic '%s' has several types associated. Only ROS topics are supported.",
        position->first.c_str());
      return "";
    }
    return position->second[0];
  }
  return "";
}

std::string Rosbag2::get_topic_type(
  std::shared_ptr<rosbag2_storage::storage_interfaces::ReadOnlyInterface> storage,
  const std::string & topic)
{
  auto all_topics_and_types = storage->get_all_topics_and_types();
  auto map_iterator_to_topic = all_topics_and_types.find(topic);

  if (map_iterator_to_topic == all_topics_and_types.end()) {
    RCUTILS_LOG_ERROR_NAMED(
      ROS_PACKAGE_NAME,
      "No messages with topic '%s' in bag file '%s'. Messages cannot be played.",
      topic.c_str(), storage->info().uri.c_str());
    return "";
  } else if (map_iterator_to_topic->second.empty()) {
    RCUTILS_LOG_ERROR_NAMED(
      ROS_PACKAGE_NAME,
      "No type specified for topic '%s' in bag file '%s'. Messages cannot be played.",
      topic.c_str(), storage->info().uri.c_str());
  }

  return map_iterator_to_topic->second;
}

void Rosbag2::record(
  const std::string & file_name,
  const std::string & topic_name,
  std::function<void(void)> after_write_action)
{
  rosbag2_storage::StorageFactory factory;
  auto storage = factory.open_read_write(file_name, "sqlite3");

  if (storage) {
    RCUTILS_LOG_INFO_NAMED(ROS_PACKAGE_NAME, "Waiting for messages...");

    auto node = std::make_shared<Rosbag2Node>("rosbag2");

    std::string type = get_topic_type(topic_name, node);

    if (type.empty()) {
      throw std::runtime_error(" Topic could not be found. Abort");
    }

    auto subscription = node->create_generic_subscription(
      topic_name,
      type,
      [storage, topic_name, after_write_action](std::shared_ptr<rmw_serialized_message_t> message) {
        auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
        bag_message->serialized_data = message;
        bag_message->topic_name = topic_name;
        rcutils_time_point_value_t time_stamp;
        int error = rcutils_system_time_now(&time_stamp);
        if (error != RCUTILS_RET_OK) {
          RCUTILS_LOG_ERROR_NAMED(
            ROS_PACKAGE_NAME,
            "Error getting current time. Error: %s", rcutils_get_error_string_safe());
        }
        bag_message->time_stamp = time_stamp;

        storage->write(bag_message);
        if (after_write_action) {
          after_write_action();
        }
      });

    if (!subscription) {
      return;
    }

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
    std::string type_name = get_topic_type(storage, topic_name);
    if (type_name.empty()) {
      return;
    }
    auto node = std::make_shared<Rosbag2Node>("rosbag2_node");
    auto publisher = node->create_generic_publisher(topic_name, type_name);

    while (storage->has_next()) {
      auto message = storage->read_next();
      // without the sleep_for() many messages are lost.
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      publisher->publish(message->serialized_data);
      RCUTILS_LOG_INFO_NAMED(ROS_PACKAGE_NAME, "published message");
    }
  }
}

}  // namespace rosbag2
