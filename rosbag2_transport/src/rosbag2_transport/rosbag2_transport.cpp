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

#include "rosbag2_transport/rosbag2_transport.hpp"

#include <chrono>
#include <map>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include "rcl/graph.h"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/time.h"

#include "rosbag2_transport/logging.hpp"
#include "rosbag2/sequential_reader.hpp"
#include "rosbag2/types.hpp"
#include "rosbag2/writer.hpp"

#include "generic_subscription.hpp"
#include "rosbag2_node.hpp"
#include "player.hpp"
#include "replayable_message.hpp"
#include "rosbag2/typesupport_helpers.hpp"

namespace rosbag2_transport
{

Rosbag2Transport::Rosbag2Transport()
  : node_(std::make_shared<Rosbag2Node>("rosbag2"))
{}

void Rosbag2Transport::init()
{
  rclcpp::init(0, nullptr);
}

void Rosbag2Transport::shutdown()
{
  rclcpp::shutdown();
}

void Rosbag2Transport::record(
  const std::string & file_name, const std::vector<std::string> & topic_names)
{
  rosbag2::Writer writer(file_name, "sqlite3");

  auto topics_and_types = node_->get_topics_with_types(topic_names);
  if (topics_and_types.empty()) {
    throw std::runtime_error("No topics found. Abort");
  }

  for (const auto & topic_and_type : topics_and_types) {
    auto topic_name = topic_and_type.first;
    auto topic_type = topic_and_type.second;

    std::shared_ptr<GenericSubscription> subscription = create_subscription(
      writer, topic_name, topic_type);

    if (subscription) {
      subscriptions_.push_back(subscription);

      writer.create_topic({topic_name, topic_type});
    }
  }

  if (subscriptions_.empty()) {
    throw std::runtime_error("No topics could be subscribed. Abort");
  }

  ROSBAG2_TRANSPORT_LOG_INFO("Waiting for messages...");
  while (rclcpp::ok()) {
    rclcpp::spin(node_);
  }
  subscriptions_.clear();
}


void Rosbag2Transport::record(const std::string & file_name)
{
  auto topics_and_types = node_->get_all_topics_with_types();
  std::vector<std::string> topic_names;
  topic_names.reserve(topics_and_types.size());
  for (const auto & topic_and_type : topics_and_types) {
    topic_names.push_back(topic_and_type.first);
  }

  if (topic_names.empty()) {
    ROSBAG2_TRANSPORT_LOG_ERROR("no topics found to record");
    return;
  }

  record(file_name, topic_names);
}

std::shared_ptr<GenericSubscription>
Rosbag2Transport::create_subscription(
  rosbag2::Writer & writer,
  const std::string & topic_name, const std::string & topic_type) const
{
  auto subscription = node_->create_generic_subscription(
    topic_name,
    topic_type,
    [&writer, topic_name](std::shared_ptr<rmw_serialized_message_t> message) {
      auto bag_message = std::make_shared<rosbag2::SerializedBagMessage>();
      bag_message->serialized_data = message;
      bag_message->topic_name = topic_name;
      rcutils_time_point_value_t time_stamp;
      int error = rcutils_system_time_now(&time_stamp);
      if (error != RCUTILS_RET_OK) {
        ROSBAG2_TRANSPORT_LOG_ERROR_STREAM(
          "Error getting current time. Error:" << rcutils_get_error_string_safe());
      }
      bag_message->time_stamp = time_stamp;

      writer.write(bag_message);
    });
  return subscription;
}

void Rosbag2Transport::play(const std::string & file_name, const Rosbag2PlayOptions & options)
{
  auto reader = std::make_shared<rosbag2::SequentialReader>(file_name, "sqlite3");

  Player player(reader);
  player.play(options);
}

}  // namespace rosbag2_transport
