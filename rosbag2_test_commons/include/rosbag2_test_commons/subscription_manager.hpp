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

#ifndef ROSBAG2_TEST_COMMONS__SUBSCRIPTION_MANAGER_HPP_
#define ROSBAG2_TEST_COMMONS__SUBSCRIPTION_MANAGER_HPP_

#include <future>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"  // rclcpp must be included before the Windows specific includes.

#include "memory_management.hpp"

namespace test_helpers
{

class SubscriptionManager
{
public:
  SubscriptionManager()
  {
    subscriber_node_ = std::make_shared<rclcpp::Node>("subscriber_node");
  }

  template<typename MessageT>
  void add_subscription(const std::string & topic_name, size_t expected_number_of_messages)
  {
    rmw_qos_profile_t qos_profile;
    qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_ALL;
    qos_profile.depth = 4;
    qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;
    qos_profile.durability = RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT;
    qos_profile.avoid_ros_namespace_conventions = false;
    expected_topics_with_size_[topic_name] = expected_number_of_messages;

    subscriptions_.push_back(
      subscriber_node_->create_subscription<MessageT>(
        topic_name,
        [this, topic_name](std::shared_ptr<rcutils_char_array_t> msg) {
          subscribed_messages_[topic_name].push_back(msg);
        }, qos_profile));
  }

  template<typename MessageT>
  std::vector<std::shared_ptr<MessageT>> get_received_messages(const std::string & topic_name)
  {
    auto messages = subscribed_messages_[topic_name];
    auto received_messages_on_topic = std::vector<std::shared_ptr<MessageT>>();
    for (const auto & msg : messages) {
      received_messages_on_topic.push_back(memory_management_.deserialize_message<MessageT>(msg));
    }
    return received_messages_on_topic;
  }

  std::future<void> spin_subscriptions()
  {
    return async(
      std::launch::async, [this]() {
        while (continue_spinning(expected_topics_with_size_)) {
          rclcpp::spin_some(subscriber_node_);
        }
      });
  }

private:
  bool continue_spinning(std::map<std::string, size_t> expected_topics_with_sizes)
  {
    for (const auto & topic_expected : expected_topics_with_sizes) {
      if (subscribed_messages_[topic_expected.first].size() < topic_expected.second) {
        return true;
      }
    }
    return false;
  }

  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  std::map<std::string, std::vector<std::shared_ptr<rcutils_char_array_t>>> subscribed_messages_;
  std::map<std::string, size_t> expected_topics_with_size_;
  rclcpp::Node::SharedPtr subscriber_node_;
  MemoryManagement memory_management_;
};

}  // namespace test_helpers

#endif  // ROSBAG2_TEST_COMMONS__SUBSCRIPTION_MANAGER_HPP_
