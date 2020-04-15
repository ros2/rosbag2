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

#ifndef ROSBAG2_TRANSPORT__RECORDER_HPP_
#define ROSBAG2_TRANSPORT__RECORDER_HPP_

#include <future>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "rclcpp/qos.hpp"

#include "rosbag2_cpp/writer.hpp"

#include "rosbag2_storage/topic_metadata.hpp"

#include "rosbag2_transport/record_options.hpp"

namespace rosbag2_cpp
{
class Writer;
}

namespace rosbag2_transport
{

class GenericSubscription;
class Rosbag2Node;

class Recorder
{
public:
  explicit Recorder(std::shared_ptr<rosbag2_cpp::Writer> writer, std::shared_ptr<Rosbag2Node> node);

  void record(const RecordOptions & record_options);

  const std::unordered_set<std::string> &
  topics_using_fallback_qos() const
  {
    return topics_warned_about_incompatibility_;
  }

  const std::unordered_map<std::string, std::shared_ptr<GenericSubscription>> &
  subscriptions() const
  {
    return subscriptions_;
  }

private:
  void topics_discovery(
    std::chrono::milliseconds topic_polling_interval,
    const std::vector<std::string> & requested_topics = {},
    bool include_hidden_topics = false);

  std::unordered_map<std::string, std::string>
  get_requested_or_available_topics(
    const std::vector<std::string> & requested_topics,
    bool include_hidden_topics = false);

  std::unordered_map<std::string, std::string>
  get_missing_topics(const std::unordered_map<std::string, std::string> & all_topics);

  void subscribe_topics(
    const std::unordered_map<std::string, std::string> & topics_and_types);

  void subscribe_topic(const rosbag2_storage::TopicMetadata & topic);

  std::shared_ptr<GenericSubscription> create_subscription(
    const std::string & topic_name, const std::string & topic_type, const rclcpp::QoS & qos);

  void record_messages() const;

  /**
   * Find the QoS profile that should be used for subscribing.
   *
   * Uses the override from record_options, if it is specified for this topic.
   * Otherwise, falls back to Rosbag2QoS::adapt_request_to_offers
   *
   *   \param topic_name The full name of the topic, with namespace (ex. /arm/joint_status).
   *   \return The QoS profile to be used for subscribing.
   */
  rclcpp::QoS subscription_qos_for_topic(const std::string & topic_name) const;

  // Serialize all currently offered QoS profiles for a topic into a YAML list.
  std::string serialized_offered_qos_profiles_for_topic(const std::string & topic_name);

  void warn_if_new_qos_for_subscribed_topic(const std::string & topic_name);

  std::shared_ptr<rosbag2_cpp::Writer> writer_;
  std::shared_ptr<Rosbag2Node> node_;
  std::unordered_map<std::string, std::shared_ptr<GenericSubscription>> subscriptions_;
  std::unordered_set<std::string> topics_warned_about_incompatibility_;
  std::string serialization_format_;
  std::unordered_map<std::string, rclcpp::QoS> topic_qos_profile_overrides_;
};

}  // namespace rosbag2_transport

#endif  // ROSBAG2_TRANSPORT__RECORDER_HPP_
