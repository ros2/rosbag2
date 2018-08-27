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
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rcl/graph.h"
#include "rclcpp/rclcpp.hpp"
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

void Rosbag2::record(
  const std::string & file_name,
  std::vector<std::string> topic_names,
  std::function<void(void)> after_write_action)
{
  rosbag2_storage::StorageFactory factory;
  auto storage = factory.open_read_write(file_name, "sqlite3");

  if (!storage) {
    throw std::runtime_error("No storage could be initialized. Abort");
    return;
  }

  auto node = std::make_shared<Rosbag2Node>("rosbag2");

  auto topics_and_types = get_topics_with_types(topic_names, node);

  if (topics_and_types.empty()) {
    throw std::runtime_error("No topics found. Abort");
  }

  for (const auto & topic_and_type : topics_and_types) {
    auto topic_name = topic_and_type.first;
    auto topic_type = topic_and_type.second;

    std::shared_ptr<GenericSubscription> subscription = create_subscription(
      after_write_action, storage, node, topic_name, topic_type);

    if (subscription) {
      subscriptions_.push_back(subscription);

      storage->create_topic(topic_name, topic_type);
    }
  }

  if (subscriptions_.empty()) {
    throw std::runtime_error("No topics could be subscribed. Abort");
    return;
  }

  RCUTILS_LOG_INFO_NAMED(ROS_PACKAGE_NAME, "Waiting for messages...");
  while (rclcpp::ok()) {
    rclcpp::spin(node);
  }
}

std::map<std::string, std::string> Rosbag2::get_topics_with_types(
  std::vector<std::string> topic_names, const std::shared_ptr<rclcpp::Node> & node)
{
  // TODO(Martin-Idel-SI): This is a short sleep to allow the node some time to discover the topic
  // This should be replaced by an auto-discovery system in the future
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  auto topics = node->get_topic_names_and_types();

  std::map<std::string, std::string> topic_names_and_types;
  for (const auto & topic_name : topic_names) {
    std::string complete_topic_name = topic_name;
    if (topic_name[0] != '/') {
      complete_topic_name = "/" + topic_name;
    }
    auto position = topics.find(complete_topic_name);
    if (position != topics.end()) {
      if (position->second.size() > 1) {
        RCUTILS_LOG_ERROR_NAMED(
          ROS_PACKAGE_NAME,
          "Topic '%s' has several types associated. Only topics with one type are supported.",
          position->first.c_str());
      } else {
        topic_names_and_types.insert({position->first, position->second[0]});
      }
    }
  }
  return topic_names_and_types;
}

std::shared_ptr<GenericSubscription>
Rosbag2::create_subscription(
  const std::function<void()> & after_write_action,
  std::shared_ptr<rosbag2_storage::storage_interfaces::ReadWriteInterface> storage,
  std::shared_ptr<Rosbag2Node> & node,
  const std::string & topic_name, const std::string & topic_type) const
{
  auto subscription = node->create_generic_subscription(
    topic_name,
    topic_type,
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
  return subscription;
}

void Rosbag2::play(const std::string & file_name)
{
  rosbag2_storage::StorageFactory factory;
  auto storage = factory.open_read_only(file_name, "sqlite3");

  if (storage) {
    auto node = std::make_shared<Rosbag2Node>("rosbag2_node");
    prepare_publishers(node, storage);

    while (storage->has_next()) {
      auto message = storage->read_next();
      // without the sleep_for() many messages are lost.
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      publishers_[message->topic_name]->publish(message->serialized_data);
      RCUTILS_LOG_INFO_NAMED(ROS_PACKAGE_NAME, "published message");
    }
  }
}

void Rosbag2::prepare_publishers(
  std::shared_ptr<Rosbag2Node> node,
  std::shared_ptr<rosbag2_storage::storage_interfaces::ReadOnlyInterface> storage)
{
  auto all_topics_and_types = storage->get_all_topics_and_types();
  for (const auto & element : all_topics_and_types) {
    publishers_.insert(std::make_pair(
      element.first, node->create_generic_publisher(element.first, element.second)));
  }
}

}  // namespace rosbag2
