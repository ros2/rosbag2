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

#include "rclcpp/rclcpp.hpp"  // rclcpp must be included before the Windows specific includes.

#include "memory_management.hpp"

namespace rosbag2_test_common
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
    auto subscription = create_subscription<MessageT>(topic_name);

    expected_topics_with_size_[topic_name] = expected_number_of_messages;
    subscriptions_.push_back(subscription);
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
  template<typename MessageT>
  rclcpp::SubscriptionBase::SharedPtr
  create_subscription(const std::string & topic_name)
  {
    using CallbackMessageT = rmw_serialized_message_t;
    using AllocT = std::allocator<void>;

    auto qos = rclcpp::QoS(rclcpp::KeepAll());
    qos.get_rmw_qos_profile().depth = 4;
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);
    qos.durability(RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT);
    qos.avoid_ros_namespace_conventions(false);
    auto options = rclcpp::SubscriptionOptionsWithAllocator<AllocT>();

    auto callback = [this, topic_name](std::shared_ptr<CallbackMessageT> msg) {
          subscribed_messages_[topic_name].push_back(msg);
    };

    auto allocator = std::make_shared<AllocT>();
    using rclcpp::AnySubscriptionCallback;
    AnySubscriptionCallback<CallbackMessageT, AllocT>
      any_subscription_callback(allocator);
    any_subscription_callback.set(callback);

    using rclcpp::message_memory_strategy::MessageMemoryStrategy;
    auto msg_mem_strat =
      MessageMemoryStrategy<CallbackMessageT, AllocT>::create_default();

    auto subscription =
      rclcpp::Subscription<CallbackMessageT, AllocT>::make_shared(
        subscriber_node_->get_node_base_interface()->get_shared_rcl_node_handle(),
        *rosidl_typesupport_cpp::get_message_type_support_handle<MessageT>(),
        topic_name,
        options.template to_rcl_subscription_options<MessageT>(qos),
        any_subscription_callback,
        options.event_callbacks,
        msg_mem_strat);

    // Intra-process message handling is not supported.
    bool use_intra_process = false;
    subscriber_node_->get_node_topics_interface()->add_subscription(
      subscription,
      use_intra_process,
      nullptr);

    auto sub_base_ptr = std::dynamic_pointer_cast<rclcpp::SubscriptionBase>(subscription);
    return sub_base_ptr;
  }

  bool continue_spinning(std::unordered_map<std::string, size_t> expected_topics_with_sizes)
  {
    for (const auto & topic_expected : expected_topics_with_sizes) {
      if (subscribed_messages_[topic_expected.first].size() < topic_expected.second) {
        return true;
      }
    }
    return false;
  }

  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  std::unordered_map<std::string,
    std::vector<std::shared_ptr<rmw_serialized_message_t>>> subscribed_messages_;
  std::unordered_map<std::string, size_t> expected_topics_with_size_;
  rclcpp::Node::SharedPtr subscriber_node_;
  MemoryManagement memory_management_;
};

}  // namespace rosbag2_test_common

#endif  // ROSBAG2_TEST_COMMON__SUBSCRIPTION_MANAGER_HPP_
