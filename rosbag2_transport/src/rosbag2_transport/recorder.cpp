// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#include "recorder.hpp"

#include <algorithm>
#include <future>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rosbag2/writer.hpp"
#include "rosbag2_transport/logging.hpp"
#include "generic_subscription.hpp"
#include "rosbag2_node.hpp"

namespace rosbag2_transport
{
Recorder::Recorder(std::shared_ptr<rosbag2::Writer> writer, std::shared_ptr<Rosbag2Node> node)
: writer_(std::move(writer)), node_(std::move(node)) {}

void Recorder::record(const RecordOptions & record_options)
{
  if (record_options.rmw_serialization_format.empty()) {
    throw std::runtime_error("No serialization format specified!");
  }
  serialization_format_ = record_options.rmw_serialization_format;
  ROSBAG2_TRANSPORT_LOG_INFO("Listening for topics...");
  subscribe_topics(get_requested_or_available_topics(record_options.topics));

  std::future<void> discovery_future;
  if (!record_options.is_discovery_disabled) {
    auto discovery = std::bind(
      &Recorder::topics_discovery, this,
      record_options.topic_polling_interval, record_options.topics);
    discovery_future = std::async(std::launch::async, discovery);
  }

  record_messages();

  if (discovery_future.valid()) {
    discovery_future.wait();
  }

  subscriptions_.clear();
}

void Recorder::topics_discovery(
  std::chrono::milliseconds topic_polling_interval,
  const std::vector<std::string> & requested_topics)
{
  while (rclcpp::ok()) {
    auto topics_to_subscribe = get_requested_or_available_topics(requested_topics);
    auto missing_topics = get_missing_topics(topics_to_subscribe);
    subscribe_topics(missing_topics);

    if (!requested_topics.empty() && subscribed_topics_.size() == requested_topics.size()) {
      ROSBAG2_TRANSPORT_LOG_INFO("All requested topics are subscribed. Stopping discovery...");
      return;
    }
    std::this_thread::sleep_for(topic_polling_interval);
  }
}

std::unordered_map<std::string, std::string>
Recorder::get_requested_or_available_topics(const std::vector<std::string> & requested_topics)
{
  return requested_topics.empty() ?
         node_->get_all_topics_with_types() :
         node_->get_topics_with_types(requested_topics);
}

std::unordered_map<std::string, std::string>
Recorder::get_missing_topics(const std::unordered_map<std::string, std::string> & topics)
{
  std::unordered_map<std::string, std::string> missing_topics;
  for (const auto & i : topics) {
    if (subscribed_topics_.find(i.first) == subscribed_topics_.end()) {
      missing_topics.emplace(i.first, i.second);
    }
  }
  return missing_topics;
}

void Recorder::subscribe_topics(
  const std::unordered_map<std::string, std::string> & topics_and_types)
{
  for (const auto & topic_with_type : topics_and_types) {
    subscribe_topic({topic_with_type.first, topic_with_type.second, serialization_format_});
  }
}

void Recorder::subscribe_topic(const rosbag2::TopicMetadata & topic)
{
  if(writer_) {
    writer_->create_topic(topic);
    subscribed_topics_.insert(topic.name);
  }

  auto subscription = create_subscription(topic.name, topic.type);

  if (subscription) {
    subscriptions_.push_back(subscription);
    ROSBAG2_TRANSPORT_LOG_INFO_STREAM("Subscribed to topic '" << topic.name << "'");
  }
  else {
    writer_->remove_topic(topic);
    subscribed_topics_.erase(topic.name);
  }

}

std::shared_ptr<GenericSubscription>
Recorder::create_subscription(
  const std::string & topic_name, const std::string & topic_type)
{
  auto subscription = node_->create_generic_subscription(
    topic_name,
    topic_type,
    [this, topic_name](std::shared_ptr<rmw_serialized_message_t> message) {
      auto bag_message = std::make_shared<rosbag2::SerializedBagMessage>();
      bag_message->serialized_data = message;
      bag_message->topic_name = topic_name;
      rcutils_time_point_value_t time_stamp;
      int error = rcutils_system_time_now(&time_stamp);
      if (error != RCUTILS_RET_OK) {
        ROSBAG2_TRANSPORT_LOG_ERROR_STREAM(
          "Error getting current time. Error:" << rcutils_get_error_string().str);
      }
      bag_message->time_stamp = time_stamp;

      writer_->write(bag_message);
    });
  return subscription;
}

void Recorder::record_messages() const
{
  spin(node_);
}

}  // namespace rosbag2_transport
