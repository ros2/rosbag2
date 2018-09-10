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
#include <queue>
#include <string>
#include <vector>

#include "rcl/graph.h"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/time.h"

#include "rosbag2/logging.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"
#include "rosbag2_storage/storage_factory.hpp"
#include "rosbag2_storage/storage_interfaces/read_only_interface.hpp"
#include "rosbag2_storage/storage_interfaces/read_write_interface.hpp"

#include "generic_subscription.hpp"
#include "rosbag2_node.hpp"
#include "player.hpp"
#include "replayable_message.hpp"
#include "typesupport_helpers.hpp"

namespace rosbag2
{

void Rosbag2::record(const std::string & file_name, const std::vector<std::string> & topic_names)
{
  auto storage = factory_.open_read_write(file_name, "sqlite3");

  if (!storage) {
    throw std::runtime_error("No storage could be initialized. Abort");
  }

  auto node = std::make_shared<Rosbag2Node>("rosbag2");

  auto topics_and_types = topic_names.empty() ?
    node->get_all_topics_with_types() :
    node->get_topics_with_types(topic_names);

  if (topics_and_types.empty()) {
    throw std::runtime_error("No topics found. Abort");
  }

  for (const auto & topic_and_type : topics_and_types) {
    auto topic_name = topic_and_type.first;
    auto topic_type = topic_and_type.second;

    std::shared_ptr<GenericSubscription> subscription = create_subscription(
      storage, node, topic_name, topic_type);

    if (subscription) {
      subscriptions_.push_back(subscription);

      storage->create_topic(topic_name, topic_type);
    }
  }

  if (subscriptions_.empty()) {
    throw std::runtime_error("No topics could be subscribed. Abort");
  }

  ROSBAG2_LOG_INFO("Waiting for messages...");
  while (rclcpp::ok()) {
    rclcpp::spin(node);
  }
  subscriptions_.clear();
}

std::shared_ptr<GenericSubscription>
Rosbag2::create_subscription(
  std::shared_ptr<rosbag2_storage::storage_interfaces::ReadWriteInterface> storage,
  std::shared_ptr<Rosbag2Node> & node,
  const std::string & topic_name, const std::string & topic_type) const
{
  auto subscription = node->create_generic_subscription(
    topic_name,
    topic_type,
    [storage, topic_name](std::shared_ptr<rmw_serialized_message_t> message) {
      auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
      bag_message->serialized_data = message;
      bag_message->topic_name = topic_name;
      rcutils_time_point_value_t time_stamp;
      int error = rcutils_system_time_now(&time_stamp);
      if (error != RCUTILS_RET_OK) {
        ROSBAG2_LOG_ERROR_STREAM(
          "Error getting current time. Error:" << rcutils_get_error_string_safe());
      }
      bag_message->time_stamp = time_stamp;

      storage->write(bag_message);
    });
  return subscription;
}

void Rosbag2::play(const std::string & file_name, Rosbag2PlayOptions options)
{
  auto storage = factory_.open_read_only(file_name, "sqlite3");
  if (!storage) {
    throw std::runtime_error("Could not open storage: " + file_name);
  }

  player_ = std::make_shared<Player>(storage);
  player_->play(options);
}

}  // namespace rosbag2
