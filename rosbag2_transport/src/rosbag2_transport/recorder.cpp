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

#include "rosbag2_cpp/writer.hpp"

#include "rosbag2_transport/logging.hpp"

#include "generic_subscription.hpp"
#include "qos.hpp"
#include "rosbag2_node.hpp"

#ifdef _WIN32
// This is necessary because of a bug in yaml-cpp's cmake
#define YAML_CPP_DLL
// This is necessary because yaml-cpp does not always use dllimport/dllexport consistently
# pragma warning(push)
# pragma warning(disable:4251)
# pragma warning(disable:4275)
#endif
#include "yaml-cpp/yaml.h"
#ifdef _WIN32
# pragma warning(pop)
#endif

namespace rosbag2_transport
{
Recorder::Recorder(std::shared_ptr<rosbag2_cpp::Writer> writer, std::shared_ptr<Rosbag2Node> node)
: writer_(std::move(writer)), node_(std::move(node)) {}

void Recorder::record(const RecordOptions & record_options)
{
  topic_qos_profile_overrides_ = record_options.topic_qos_profile_overrides;
  if (record_options.rmw_serialization_format.empty()) {
    throw std::runtime_error("No serialization format specified!");
  }
  serialization_format_ = record_options.rmw_serialization_format;
  ROSBAG2_TRANSPORT_LOG_INFO("Listening for topics...");
  subscribe_topics(
    get_requested_or_available_topics(record_options.topics, record_options.include_hidden_topics));

  std::future<void> discovery_future;
  if (!record_options.is_discovery_disabled) {
    auto discovery = std::bind(
      &Recorder::topics_discovery, this,
      record_options.topic_polling_interval,
      record_options.topics,
      record_options.include_hidden_topics);
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
  const std::vector<std::string> & requested_topics,
  bool include_hidden_topics)
{
  while (rclcpp::ok()) {
    auto topics_to_subscribe =
      get_requested_or_available_topics(requested_topics, include_hidden_topics);
    for (const auto & topic_and_type : topics_to_subscribe) {
      warn_if_new_qos_for_subscribed_topic(topic_and_type.first);
    }
    auto missing_topics = get_missing_topics(topics_to_subscribe);
    subscribe_topics(missing_topics);

    if (!requested_topics.empty() && subscriptions_.size() == requested_topics.size()) {
      ROSBAG2_TRANSPORT_LOG_INFO("All requested topics are subscribed. Stopping discovery...");
      return;
    }
    std::this_thread::sleep_for(topic_polling_interval);
  }
}

std::unordered_map<std::string, std::string>
Recorder::get_requested_or_available_topics(
  const std::vector<std::string> & requested_topics,
  bool include_hidden_topics)
{
  return requested_topics.empty() ?
         node_->get_all_topics_with_types(include_hidden_topics) :
         node_->get_topics_with_types(requested_topics);
}

std::unordered_map<std::string, std::string>
Recorder::get_missing_topics(const std::unordered_map<std::string, std::string> & all_topics)
{
  std::unordered_map<std::string, std::string> missing_topics;
  for (const auto & i : all_topics) {
    if (subscriptions_.find(i.first) == subscriptions_.end()) {
      missing_topics.emplace(i.first, i.second);
    }
  }
  return missing_topics;
}


void Recorder::subscribe_topics(
  const std::unordered_map<std::string, std::string> & topics_and_types)
{
  for (const auto & topic_with_type : topics_and_types) {
    subscribe_topic(
      {
        topic_with_type.first,
        topic_with_type.second,
        serialization_format_,
        serialized_offered_qos_profiles_for_topic(topic_with_type.first)
      });
  }
}

void Recorder::subscribe_topic(const rosbag2_storage::TopicMetadata & topic)
{
  // Need to create topic in writer before we are trying to create subscription. Since in
  // callback for subscription we are calling writer_->write(bag_message); and it could happened
  // that callback called before we reached out the line: writer_->create_topic(topic)
  writer_->create_topic(topic);

  Rosbag2QoS subscription_qos{subscription_qos_for_topic(topic.name)};
  auto subscription = create_subscription(topic.name, topic.type, subscription_qos);
  if (subscription) {
    subscriptions_.insert({topic.name, subscription});
    ROSBAG2_TRANSPORT_LOG_INFO_STREAM("Subscribed to topic '" << topic.name << "'");
  } else {
    writer_->remove_topic(topic);
    subscriptions_.erase(topic.name);
  }
}

std::shared_ptr<GenericSubscription>
Recorder::create_subscription(
  const std::string & topic_name, const std::string & topic_type, const rclcpp::QoS & qos)
{
  auto subscription = node_->create_generic_subscription(
    topic_name,
    topic_type,
    qos,
    [this, topic_name](std::shared_ptr<rmw_serialized_message_t> message) {
      auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
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

std::string Recorder::serialized_offered_qos_profiles_for_topic(const std::string & topic_name)
{
  YAML::Node offered_qos_profiles;
  auto endpoints = node_->get_publishers_info_by_topic(topic_name);
  for (const auto & info : endpoints) {
    offered_qos_profiles.push_back(Rosbag2QoS(info.qos_profile()));
  }
  return YAML::Dump(offered_qos_profiles);
}

rclcpp::QoS Recorder::subscription_qos_for_topic(const std::string & topic_name) const
{
  if (topic_qos_profile_overrides_.count(topic_name)) {
    ROSBAG2_TRANSPORT_LOG_INFO_STREAM("Overriding subscription profile for " << topic_name);
    return topic_qos_profile_overrides_.at(topic_name);
  } else {
    return adapt_qos_to_publishers(topic_name);
  }
  return adapt_qos_to_publishers(topic_name);
}

rclcpp::QoS Recorder::adapt_qos_to_publishers(const std::string & topic_name) const
{
  auto endpoints = node_->get_publishers_info_by_topic(topic_name);
  size_t num_endpoints = endpoints.size();
  size_t reliability_reliable_endpoints_count = 0;
  size_t durability_transient_local_endpoints_count = 0;
  for (const auto & endpoint : endpoints) {
    const auto & profile = endpoint.qos_profile().get_rmw_qos_profile();
    if (profile.reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE) {
      reliability_reliable_endpoints_count++;
    }
    if (profile.durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL) {
      durability_transient_local_endpoints_count++;
    }
  }

  // We set policies in order as defined in rmw_qos_profile_t
  Rosbag2QoS request_qos;
  // Policy: history, depth
  // History does not affect compatibility
  request_qos.default_history();

  // Policy: reliability
  if (reliability_reliable_endpoints_count == num_endpoints) {
    request_qos.reliable();
  } else {
    if (reliability_reliable_endpoints_count > 0) {
      ROSBAG2_TRANSPORT_LOG_WARN_STREAM(
        "Some, but not all, publishers on topic \"" << topic_name << "\" "
          "are offering RMW_QOS_POLICY_RELIABILITY_RELIABLE. "
          "Falling back to RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT "
          "as it will connect to all publishers. "
          "Some messages from Reliable publishers could be dropped.");
    }
    request_qos.best_effort();
  }

  // Policy: durability
  // If all publishers offer transient_local, we can request it and receive latched messages
  if (durability_transient_local_endpoints_count == num_endpoints) {
    request_qos.transient_local();
  } else {
    if (durability_transient_local_endpoints_count > 0) {
      ROSBAG2_TRANSPORT_LOG_WARN_STREAM(
        "Some, but not all, publishers on topic \"" << topic_name << "\" "
          "are offering RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL. "
          "Falling back to RMW_QOS_POLICY_DURABILITY_VOLATILE "
          "as it will connect to all publishers. "
          "Previously-published latched messages will not be retrieved.");
    }
    request_qos.durability_volatile();
  }
  // Policy: deadline
  // Deadline does not affect delivery of messages,
  // and we do not record Deadline"Missed events.
  // We can always use unspecified deadline, which will be compatible with all publishers.

  // Policy: lifespan
  // Lifespan does not affect compatibiliy

  // Policy: liveliness, liveliness_lease_duration
  // Liveliness does not affect delivery of messages,
  // and we do not record LivelinessChanged events.
  // We can always use unspecified liveliness, which will be compatible with all publishers.
  return request_qos;
}

void Recorder::warn_if_new_qos_for_subscribed_topic(const std::string & topic_name)
{
  auto existing_subscription = subscriptions_.find(topic_name);
  if (existing_subscription == subscriptions_.end()) {
    // Not subscribed yet
    return;
  }
  if (topics_warned_about_incompatibility_.count(topic_name) > 0) {
    // Already warned about this topic
    return;
  }
  const auto & used_profile = existing_subscription->second->qos_profile().get_rmw_qos_profile();
  auto publishers_info = node_->get_publishers_info_by_topic(topic_name);
  for (const auto & info : publishers_info) {
    auto new_profile = info.qos_profile().get_rmw_qos_profile();
    bool incompatible_reliability =
      new_profile.reliability == RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT &&
      used_profile.reliability != RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    bool incompatible_durability =
      new_profile.durability == RMW_QOS_POLICY_DURABILITY_VOLATILE &&
      used_profile.durability != RMW_QOS_POLICY_DURABILITY_VOLATILE;

    if (incompatible_reliability) {
      ROSBAG2_TRANSPORT_LOG_WARN_STREAM(
        "A new publisher for subscribed topic " << topic_name << " "
          "was found offering RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT, "
          "but rosbag already subscribed requesting RMW_QOS_POLICY_RELIABILITY_RELIABLE. "
          "Messages from this new publisher will not be recorded.");
      topics_warned_about_incompatibility_.insert(topic_name);
    } else if (incompatible_durability) {
      ROSBAG2_TRANSPORT_LOG_WARN_STREAM(
        "A new publisher for susbcribed topic " << topic_name << " "
          "was found offering RMW_QOS_POLICY_DURABILITY_VOLATILE, "
          "but rosbag2 already subscribed requesting RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL. "
          "Messages from this new publisher will not be recorded.");
      topics_warned_about_incompatibility_.insert(topic_name);
    }
  }
}

}  // namespace rosbag2_transport
