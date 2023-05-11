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

#ifndef ROSBAG2_TEST_COMMON__SUBSCRIPTION_MANAGER_HPP_
#define ROSBAG2_TEST_COMMON__SUBSCRIPTION_MANAGER_HPP_

#include <future>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/clock.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/intra_process_setting.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/publisher_base.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rclcpp/subscription_base.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rclcpp/utilities.hpp"

namespace rosbag2_test_common
{

class SubscriptionManager
{
public:
  SubscriptionManager()
  {
    subscriber_node_ = std::make_shared<rclcpp::Node>(
      "subscription_manager_" + std::to_string(rclcpp::Clock().now().nanoseconds()),
      rclcpp::NodeOptions().start_parameter_event_publisher(false).enable_rosout(false)
    );
  }

  template<typename MessageT>
  void add_subscription(
    const std::string & topic_name,
    size_t expected_number_of_messages,
    const rclcpp::QoS & qos = rclcpp::QoS{rmw_qos_profile_default.depth})
  {
    expected_topics_with_size_[topic_name] = expected_number_of_messages;

    auto options = rclcpp::SubscriptionOptions();
    options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;

    subscriptions_[topic_name] =
      subscriber_node_->create_subscription<MessageT>(
      topic_name,
      qos,
      [this, topic_name](std::shared_ptr<rclcpp::SerializedMessage> msg) {
        subscribed_messages_[topic_name].push_back(msg);
      },
      options);
  }

  template<typename MessageT>
  std::vector<std::shared_ptr<MessageT>> get_received_messages(const std::string & topic_name)
  {
    static rclcpp::Serialization<MessageT> serializer;
    auto messages = subscribed_messages_[topic_name];
    auto received_messages_on_topic = std::vector<std::shared_ptr<MessageT>>();
    for (const auto & serialized_msg : messages) {
      auto msg = std::make_shared<MessageT>();
      serializer.deserialize_message(serialized_msg.get(), msg.get());
      received_messages_on_topic.push_back(msg);
    }
    return received_messages_on_topic;
  }

  /// \brief Wait until publishers will be connected to the subscribers or timeout occur.
  /// \tparam Timeout Data type for timeout duration from std::chrono:: namespace
  /// \param publishers List of raw pointers to the publishers
  /// \param timeout Maximum time duration during which discovery should happen.
  /// \param n_subscribers_to_match Number of subscribers each publisher should have for match.
  /// \return true if publishers have specified number of subscribers, otherwise false.
  template<typename Timeout>
  bool spin_and_wait_for_matched(
    const std::vector<rclcpp::PublisherBase *> & publishers,
    Timeout timeout, size_t n_subscribers_to_match = 1)
  {
    // Sanity check that we have valid input
    for (const auto publisher_ptr : publishers) {
      if (publisher_ptr == nullptr) {
        throw std::invalid_argument("Null pointer in publisher list");
      }
      std::string topic_name{publisher_ptr->get_topic_name()};
      if (expected_topics_with_size_.find(topic_name) == expected_topics_with_size_.end()) {
        throw std::invalid_argument(
                "Publisher's topic name = `" + topic_name + "` not found in expected topics list");
      }
    }

    using clock = std::chrono::steady_clock;
    auto start = clock::now();

    rclcpp::executors::SingleThreadedExecutor exec;
    bool matched = false;
    while (!matched && ((clock::now() - start) < timeout)) {
      exec.spin_node_some(subscriber_node_);
      matched = true;
      for (const auto publisher_ptr : publishers) {
        if (publisher_ptr->get_subscription_count() +
          publisher_ptr->get_intra_process_subscription_count() < n_subscribers_to_match)
        {
          matched = false;
          break;
        }
      }
      // Sleep for a few milliseconds to avoid busy loop
      std::this_thread::sleep_for(sleep_per_loop_);
    }
    return matched;
  }

  /// @brief Wait until publishers will be connected to the subscriptions or timeout occur.
  /// @param topic_names List of topic names
  /// @param timeout Maximum time duration during which discovery should happen.
  /// @param n_publishers_to_match Number of publishers each subscription should have for match.
  /// @return true if subscriptions have specified number of publishers, otherwise false.
  bool spin_and_wait_for_matched(
    const std::vector<std::string> & topic_names,
    std::chrono::duration<double> timeout = std::chrono::seconds(10),
    size_t n_publishers_to_match = 1)
  {
    // Sanity check that we have valid input
    if (topic_names.empty()) {
      throw std::invalid_argument("List of topic names is empty");
    }
    for (const auto & topic_name : topic_names) {
      if (subscriptions_.count(topic_name) == 0) {
        throw std::invalid_argument(
                "Publisher's topic name = `" + topic_name + "` not found in expected topics list");
      }
    }

    using clock = std::chrono::steady_clock;
    auto start = clock::now();

    rclcpp::executors::SingleThreadedExecutor exec;
    bool matched = false;
    while (!matched && ((clock::now() - start) < timeout)) {
      exec.spin_node_some(subscriber_node_);
      matched = true;
      for (const auto & topic_name : topic_names) {
        if (subscriptions_.find(topic_name) == subscriptions_.end() ||
          subscriptions_[topic_name]->get_publisher_count() < n_publishers_to_match)
        {
          matched = false;
          break;
        }
      }
      // Sleep for a few milliseconds to avoid busy loop
      std::this_thread::sleep_for(sleep_per_loop_);
    }
    return matched;
  }

  std::future<void> spin_subscriptions(
    std::chrono::duration<double> timeout = std::chrono::seconds(10))
  {
    return async(
      std::launch::async, [this, timeout]() {
        rclcpp::executors::SingleThreadedExecutor exec;
        auto start = std::chrono::high_resolution_clock::now();
        while (rclcpp::ok() && continue_spinning(expected_topics_with_size_, start, timeout)) {
          exec.spin_node_some(subscriber_node_);
        }
      });
  }

  void spin_subscriptions_sync()
  {
    rclcpp::executors::SingleThreadedExecutor exec;
    auto start = std::chrono::high_resolution_clock::now();
    while (rclcpp::ok() && continue_spinning(expected_topics_with_size_, start)) {
      exec.spin_node_some(subscriber_node_);
    }
  }

private:
  bool continue_spinning(
    const std::unordered_map<std::string, size_t> & expected_topics_with_sizes,
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time,
    std::chrono::duration<double> timeout = std::chrono::seconds(10))
  {
    auto current = std::chrono::high_resolution_clock::now();
    if (current - start_time > timeout) {
      RCLCPP_WARN(
        subscriber_node_->get_logger(),
        "SubscriptionManager::continue_spinning(..) finished by timeout");
      return false;
    }

    for (const auto & topic_expected : expected_topics_with_sizes) {
      if (subscribed_messages_[topic_expected.first].size() < topic_expected.second) {
        // Sleep for a few milliseconds to avoid busy loop
        std::this_thread::sleep_for(sleep_per_loop_);
        return true;
      }
    }
    return false;
  }

  std::unordered_map<std::string, rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  std::unordered_map<std::string,
    std::vector<std::shared_ptr<rclcpp::SerializedMessage>>> subscribed_messages_;
  std::unordered_map<std::string, size_t> expected_topics_with_size_;
  rclcpp::Node::SharedPtr subscriber_node_;
  const std::chrono::milliseconds sleep_per_loop_ = std::chrono::milliseconds(2);
};

}  // namespace rosbag2_test_common

#endif  // ROSBAG2_TEST_COMMON__SUBSCRIPTION_MANAGER_HPP_
