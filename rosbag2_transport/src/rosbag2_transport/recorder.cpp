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

#include <memory>
#include <string>
#include <utility>

#include "rosbag2_transport/logging.hpp"

namespace rosbag2_transport
{
Recorder::Recorder(std::shared_ptr<rosbag2::Writer> writer, std::shared_ptr<Rosbag2Node> node)
: writer_(std::move(writer)), node_(std::move(node)) {}

void Recorder::record(const RecordOptions & record_options)
{
  auto topics_and_types = record_options.all ?
    node_->get_all_topics_with_types() :
    node_->get_topics_with_types(record_options.topics);

  if (topics_and_types.empty()) {
    throw std::runtime_error("No topics found. Abort");
  }

  for (const auto & topic_and_type : topics_and_types) {
    auto topic_name = topic_and_type.first;
    auto topic_type = topic_and_type.second;

    auto subscription = create_subscription(topic_name, topic_type);
    if (subscription) {
      subscriptions_.push_back(subscription);
      writer_->create_topic({topic_name, topic_type});
      ROSBAG2_TRANSPORT_LOG_INFO_STREAM("Subscribed to topic '" << topic_name << "'");
    }
  }

  if (subscriptions_.empty()) {
    throw std::runtime_error("No topics could be subscribed. Abort");
  }

  ROSBAG2_TRANSPORT_LOG_INFO("Subscription setup complete.");
  rclcpp::spin(node_);
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
          "Error getting current time. Error:" << rcutils_get_error_string_safe());
      }
      bag_message->time_stamp = time_stamp;

      writer_->write(bag_message);
    });
  return subscription;
}

}  // namespace rosbag2_transport
