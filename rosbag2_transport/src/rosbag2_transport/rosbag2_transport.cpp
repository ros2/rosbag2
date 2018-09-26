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
#include <utility>
#include <vector>

#include "rcl/graph.h"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/time.h"

#include "rosbag2_transport/logging.hpp"
#include "rosbag2/sequential_reader.hpp"
#include "rosbag2/types.hpp"
#include "rosbag2/typesupport_helpers.hpp"
#include "rosbag2/writer.hpp"

#include "generic_subscription.hpp"
#include "player.hpp"
#include "replayable_message.hpp"
#include "rosbag2_node.hpp"

namespace rosbag2_transport
{

Rosbag2Transport::Rosbag2Transport(std::shared_ptr<rosbag2::Rosbag2Factory> factory)
: factory_(std::move(factory)) {}

void Rosbag2Transport::init()
{
  rclcpp::init(0, nullptr);
}

void Rosbag2Transport::shutdown()
{
  rclcpp::shutdown();
}

void Rosbag2Transport::record(
  const StorageOptions & storage_options, const RecordOptions & record_options)
{
  auto writer = factory_->create_writer(storage_options);

  auto transport_node = setup_node();

  auto topics_and_types = record_options.all ?
    transport_node->get_all_topics_with_types() :
    transport_node->get_topics_with_types(record_options.topics);

  if (topics_and_types.empty()) {
    throw std::runtime_error("No topics found. Abort");
  }

  for (const auto & topic_and_type : topics_and_types) {
    auto topic_name = topic_and_type.first;
    auto topic_type = topic_and_type.second;

    auto subscription = create_subscription(transport_node, topic_name, topic_type, writer);
    if (subscription) {
      subscriptions_.push_back(subscription);
      writer->create_topic({topic_name, topic_type});
    }
  }

  if (subscriptions_.empty()) {
    throw std::runtime_error("No topics could be subscribed. Abort");
  }

  ROSBAG2_TRANSPORT_LOG_INFO("Subscription setup complete.");
  rclcpp::spin(transport_node);
  subscriptions_.clear();
}

std::shared_ptr<Rosbag2Node> Rosbag2Transport::setup_node()
{
  if (!transport_node_) {
    transport_node_ = std::make_shared<Rosbag2Node>("rosbag2");
  }
  return transport_node_;
}

std::shared_ptr<GenericSubscription>
Rosbag2Transport::create_subscription(
  std::shared_ptr<Rosbag2Node> & node,
  const std::string & topic_name, const std::string & topic_type,
  std::shared_ptr<rosbag2::Writer> writer) const
{
  auto subscription = node->create_generic_subscription(
    topic_name,
    topic_type,
    [writer, topic_name](std::shared_ptr<rmw_serialized_message_t> message) {
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

      writer->write(bag_message);
    });
  return subscription;
}

void Rosbag2Transport::play(
  const StorageOptions & storage_options, const PlayOptions & play_options)
{
  auto reader = factory_->create_sequential_reader(storage_options);
  auto transport_node = setup_node();

  Player player(reader, transport_node);
  player.play(play_options);
}

void Rosbag2Transport::print_bag_info(const std::string & uri)
{
  auto metadata = factory_->create_info()->read_metadata(uri);
  std::stringstream ss;
  ss << "Storage identifier:  " << metadata.storage_identifier << "\n";
  ss << "File encoding:       " << metadata.encoding << "\n";
  ss << "Associated files (relative paths):\n";
  for (const auto & file : metadata.relative_file_paths) {
    ss << "        - " << file << "\n";
  }
  ss << "Starting time:       " << metadata.starting_time.time_since_epoch().count() << "\n";
  ss << "End time:            " <<
  (metadata.starting_time.time_since_epoch() + metadata.duration).count() << "\n";
  ss << "Duration:            " << metadata.duration.count() << "\n";
  ss << "Total message count: " << metadata.message_count << "\n";
  ss << "Topics with Type and message count:\n";
  for (const auto & topic_with_type_and_count : metadata.topics_with_message_count) {
    ss << "        - " << topic_with_type_and_count.topic_with_type.name <<
      " ; " << topic_with_type_and_count.topic_with_type.type <<
      " ; " << topic_with_type_and_count.message_count << "\n";
  }

  std::cout << ss.str() << std::endl;
}

}  // namespace rosbag2_transport
