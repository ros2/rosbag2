// Copyright 2025 Apex.AI, Inc.
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


#include <gmock/gmock.h>

#include <iostream>
#include <thread>

#include "rosbag2_interfaces/msg/write_split_event.hpp"
#include "rosbag2_transport/recorder_event_notifier.hpp"
#include "rosbag2_test_common/subscription_manager.hpp"

using namespace ::testing;          // NOLINT
using namespace rosbag2_transport;  // NOLINT
using namespace rosbag2_test_common;  // NOLINT

class TestRecorderEventNotifier : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    node_ = std::make_unique<rclcpp::Node>("test_recorder_event_notifier");
    notifier_ = std::make_unique<RecorderEventNotifier>(node_.get());
  }

  std::unique_ptr<rclcpp::Node> node_;
  std::unique_ptr<RecorderEventNotifier> notifier_;
};

TEST_F(TestRecorderEventNotifier, default_ctor_dtor)
{
  // Destructor should not throw
  ASSERT_NO_THROW(notifier_.reset());
}

TEST_F(TestRecorderEventNotifier, handle_bag_split_event)
{
  // Disable statistics publishing
  notifier_->set_messages_lost_statistics_max_publishing_rate(0.0f);
  const size_t expected_number_of_messages = 2;
  const std::string topic_name = "events/write_split";
  auto sub = std::make_unique<SubscriptionManager>();
  rclcpp::QoS sub_qos(rclcpp::QoS{10}.reliability(rclcpp::ReliabilityPolicy::Reliable));
  // Create a subscription to the write_split event
  sub->add_subscription<rosbag2_interfaces::msg::WriteSplitEvent>(
    topic_name, expected_number_of_messages, sub_qos
  );
  // Wait for discovery to match publishers with subscribers
  ASSERT_TRUE(sub->spin_and_wait_for_matched({topic_name}, std::chrono::seconds(30), 1));
  auto await_received_messages = sub->spin_subscriptions(std::chrono::seconds(30));

  rosbag2_cpp::bag_events::BagSplitInfo bag_split_info1;
  bag_split_info1.closed_file = "closed_file1.bag";
  bag_split_info1.opened_file = "opened_file2.bag";

  rosbag2_cpp::bag_events::BagSplitInfo bag_split_info2;
  bag_split_info2.closed_file = "closed_file2.bag";
  bag_split_info2.opened_file = "";

  ASSERT_NO_THROW(notifier_->on_bag_split_in_recorder(bag_split_info1));
  ASSERT_NO_THROW(notifier_->on_bag_split_in_recorder(bag_split_info2));

  await_received_messages.get();
  auto received_split_event_messages =
    sub->get_received_messages<rosbag2_interfaces::msg::WriteSplitEvent>(topic_name);
  ASSERT_THAT(received_split_event_messages, SizeIs(expected_number_of_messages));

  EXPECT_THAT(received_split_event_messages[0]->closed_file, Eq(bag_split_info1.closed_file));
  EXPECT_THAT(received_split_event_messages[0]->opened_file, Eq(bag_split_info1.opened_file));
  EXPECT_THAT(received_split_event_messages[1]->closed_file, Eq(bag_split_info2.closed_file));
  EXPECT_THAT(received_split_event_messages[1]->opened_file, Eq(bag_split_info2.opened_file));
}

TEST_F(TestRecorderEventNotifier, messages_lost_in_transport_correctly_accumulated)
{
  rclcpp::QOSMessageLostInfo qos_msgs_lost_info;
  qos_msgs_lost_info.total_count = 20;
  qos_msgs_lost_info.total_count_change = 5;
  ASSERT_NO_THROW(notifier_->on_messages_lost_in_transport("topic1", qos_msgs_lost_info));
  ASSERT_NO_THROW(notifier_->on_messages_lost_in_transport("topic2", qos_msgs_lost_info));
  EXPECT_EQ(notifier_->get_total_num_messages_lost_in_transport(), 10u);
}

TEST_F(TestRecorderEventNotifier, reset_messages_lost_counters)
{
  rclcpp::QOSMessageLostInfo qos_msgs_lost_info;
  qos_msgs_lost_info.total_count_change = 3;
  ASSERT_NO_THROW(notifier_->on_messages_lost_in_transport("topic2", qos_msgs_lost_info));

  EXPECT_EQ(notifier_->get_total_num_messages_lost_in_transport(), 3u);

  notifier_->reset_total_num_messages_lost_in_transport();

  EXPECT_EQ(notifier_->get_total_num_messages_lost_in_transport(), 0u);
}

TEST_F(TestRecorderEventNotifier, set_statistics_publishing_rate)
{
  ASSERT_NO_THROW(notifier_->set_messages_lost_statistics_max_publishing_rate(1.0f));
  ASSERT_NO_THROW(notifier_->set_messages_lost_statistics_max_publishing_rate(0.0f));
  ASSERT_THROW(notifier_->set_messages_lost_statistics_max_publishing_rate(-1.0f),
               std::invalid_argument);
  ASSERT_NO_THROW(notifier_->set_messages_lost_statistics_max_publishing_rate(0.0f));
  ASSERT_THROW(notifier_->set_messages_lost_statistics_max_publishing_rate(-10.0f),
               std::invalid_argument);
  ASSERT_THROW(notifier_->set_messages_lost_statistics_max_publishing_rate(1000.0f),
               std::invalid_argument);
  ASSERT_THROW(notifier_->set_messages_lost_statistics_max_publishing_rate(1000.001f),
               std::invalid_argument);
  ASSERT_NO_THROW(notifier_->set_messages_lost_statistics_max_publishing_rate(999.999f));
}

TEST_F(TestRecorderEventNotifier, zero_messages_lost)
{
  rclcpp::QOSMessageLostInfo qos_msgs_lost_info;
  qos_msgs_lost_info.total_count = 0;
  qos_msgs_lost_info.total_count_change = 0;
  ASSERT_NO_THROW(notifier_->on_messages_lost_in_transport("topic1", qos_msgs_lost_info));

  EXPECT_EQ(notifier_->get_total_num_messages_lost_in_transport(), 0u);
}

TEST_F(TestRecorderEventNotifier, large_message_counts)
{
  rclcpp::QOSMessageLostInfo qos_msgs_lost_info;
  qos_msgs_lost_info.total_count = 1000000;
  qos_msgs_lost_info.total_count_change = 1000000;
  notifier_->on_messages_lost_in_transport("topic1", qos_msgs_lost_info);

  EXPECT_EQ(notifier_->get_total_num_messages_lost_in_transport(), 1000000u);
}

TEST_F(TestRecorderEventNotifier, reset_between_message_loss_notifications)
{
  rclcpp::QOSMessageLostInfo qos_msgs_lost_info;
  qos_msgs_lost_info.total_count = 1000000;
  qos_msgs_lost_info.total_count_change = 10;
  notifier_->on_messages_lost_in_transport("topic1", qos_msgs_lost_info);

  notifier_->reset_total_num_messages_lost_in_transport();

  qos_msgs_lost_info.total_count_change = 7;
  notifier_->on_messages_lost_in_transport("topic2", qos_msgs_lost_info);

  EXPECT_EQ(notifier_->get_total_num_messages_lost_in_transport(), 7u);
}

TEST_F(TestRecorderEventNotifier, thread_safety_with_concurrent_access)
{
  constexpr size_t num_threads_for_each_event = 10;
  constexpr size_t iterations_per_thread = 1000;
  constexpr size_t expected_transport_lost = num_threads_for_each_event * iterations_per_thread;

  std::vector<std::thread> threads;

  // Simulate concurrent access to on_messages_lost_in_transport
  for (size_t i = 0; i < num_threads_for_each_event; i++) {
    // Simulate concurrent access to on_messages_lost_in_transport
    threads.emplace_back([this, i, iterations_per_thread]() {
        for (size_t j = 0; j < iterations_per_thread; j++) {
          rclcpp::QOSMessageLostInfo qos_msgs_lost_info;
          qos_msgs_lost_info.total_count_change = 1;
          notifier_->on_messages_lost_in_transport("topic" + std::to_string(i), qos_msgs_lost_info);
        }
    });
  }

  for (auto & thread : threads) {
    thread.join();
  }

  // Verify that the total counts are consistent
  EXPECT_EQ(notifier_->get_total_num_messages_lost_in_transport(), expected_transport_lost);
}
