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
  ROSBAG2_TRANSPORT_LOG_INFO("Setup complete. Listening for topics...");
  serialization_format_ = record_options.rmw_serialization_format;
  auto discovery_future = launch_topics_discovery(
    record_options.topic_polling_interval, record_options.topics);

  record_messages();

  discovery_future.wait();
  subscriptions_.clear();
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

std::future<void> Recorder::launch_topics_discovery(
  std::chrono::milliseconds topic_polling_interval,
  const std::vector<std::string> & topics_to_record)
{
  auto subscribe_to_topics = [this, topics_to_record, topic_polling_interval] {
      while (rclcpp::ok()) {
        auto all_topics_and_types_to_subscribe = topics_to_record.empty() ?
          node_->get_all_topics_with_types() :
          node_->get_topics_with_types(topics_to_record);

        if (is_every_topic_subscribed(topics_to_record)) {
          return;
        }

        subscribe_all_missing_topics(all_topics_and_types_to_subscribe);
        std::this_thread::sleep_for(topic_polling_interval);
      }
    };

  return std::async(std::launch::async, subscribe_to_topics);
}

bool Recorder::is_every_topic_subscribed(
  const std::vector<std::string> & topics_to_record) const
{
  return !topics_to_record.empty() && subscriptions_.size() == topics_to_record.size();
}

void Recorder::subscribe_all_missing_topics(
  const std::unordered_map<std::string, std::string> & all_topics_and_types)
{
  for (const auto & topic_with_type : all_topics_and_types) {
    bool already_subscribed = find(
      subscribed_topics_.begin(),
      subscribed_topics_.end(),
      topic_with_type.first) != subscribed_topics_.end();

    if (!already_subscribed) {
      subscribe_topic({topic_with_type.first, topic_with_type.second, serialization_format_});
    }
  }
}

void Recorder::subscribe_topic(const rosbag2::TopicMetadata & topic)
{
  auto subscription = create_subscription(topic.name, topic.type);
  if (subscription) {
    subscribed_topics_.push_back(topic.name);
    subscriptions_.push_back(subscription);
    writer_->create_topic(topic);
    ROSBAG2_TRANSPORT_LOG_INFO_STREAM("Subscribed to topic '" << topic.name << "'");
  }
}

void Recorder::record_messages() const
{
  spin(node_);
}

}  // namespace rosbag2_transport
