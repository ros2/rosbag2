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

#include <thread>
#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <utility>
#include <queue>
#include <mutex>

#include "rosbag2_interfaces/msg/messages_lost_event.hpp"
#include "rosbag2_interfaces/msg/messages_lost_event_topic_stat.hpp"
#include "rosbag2_interfaces/msg/write_split_event.hpp"
#include "rosbag2_transport/recorder_event_notifier.hpp"
#include "rosbag2_transport/rclcpp_publisher_wrapper.hpp"
#include "rosbag2_test_common/subscription_manager.hpp"

using namespace ::testing;          // NOLINT
using namespace rosbag2_transport;  // NOLINT
using namespace rosbag2_test_common;  // NOLINT

using WriteSplitEvent = rosbag2_interfaces::msg::WriteSplitEvent;
using MessagesLostEvent = rosbag2_interfaces::msg::MessagesLostEvent;

template<typename T>
class MockPublisherWrapper : public RclcppPublisherWrapper<T> {
public:
  explicit MockPublisherWrapper(typename rclcpp::Publisher<T>::SharedPtr publisher)
  :RclcppPublisherWrapper<T>(std::move(publisher)) {}

  MockPublisherWrapper(const MockPublisherWrapper & other)
  :RclcppPublisherWrapper<T>(other.publisher_) {}

  ~MockPublisherWrapper() override = default;

  MOCK_METHOD(void, publish, (const T & msg), (override));
};

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

TEST_F(TestRecorderEventNotifier, get_default_events_topic_names)
{
  EXPECT_STREQ(RecorderEventNotifier::get_default_write_split_topic_name(),
               "events/write_split");
  EXPECT_STREQ(RecorderEventNotifier::get_default_messages_lost_topic_name(),
               "events/rosbag2_messages_lost");
}

TEST_F(TestRecorderEventNotifier, get_current_events_topic_names)
{
  const auto fqn_default_write_split =
    node_->get_node_base_interface()->resolve_topic_or_service_name(
      RecorderEventNotifier::get_default_write_split_topic_name(),
      /*is_service=*/false,
      /*only_expand=*/false);

  const auto fqn_default_msgs_lost =
    node_->get_node_base_interface()->resolve_topic_or_service_name(
      RecorderEventNotifier::get_default_messages_lost_topic_name(),
      /*is_service=*/false,
      /*only_expand=*/false);

  EXPECT_STREQ(notifier_->get_write_split_topic_name().data(), fqn_default_write_split.c_str());
  EXPECT_STREQ(notifier_->get_messages_lost_topic_name().data(), fqn_default_msgs_lost.c_str());
}

TEST_F(TestRecorderEventNotifier, can_handle_and_publish_bag_split_events)
{
  // Disable statistics publishing
  notifier_->set_messages_lost_statistics_max_publishing_rate(0.0f);
  const size_t expected_number_of_messages = 2;
  const std::string topic_name = RecorderEventNotifier::get_default_write_split_topic_name();
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

  EXPECT_THAT(received_split_event_messages[0]->node_name, Eq(node_->get_fully_qualified_name()));
  EXPECT_THAT(received_split_event_messages[0]->closed_file, Eq(bag_split_info1.closed_file));
  EXPECT_THAT(received_split_event_messages[0]->opened_file, Eq(bag_split_info1.opened_file));
  EXPECT_THAT(received_split_event_messages[1]->closed_file, Eq(bag_split_info2.closed_file));
  EXPECT_THAT(received_split_event_messages[1]->opened_file, Eq(bag_split_info2.opened_file));
}

TEST_F(TestRecorderEventNotifier, can_publish_event_on_messages_lost_in_recorder)
{
  using MessagesLostEventTopicStat = rosbag2_interfaces::msg::MessagesLostEventTopicStat;
  notifier_->set_messages_lost_statistics_max_publishing_rate(30.0f);
  const size_t expected_number_of_messages = 1;
  const std::string topic_name = RecorderEventNotifier::get_default_messages_lost_topic_name();
  auto sub = std::make_unique<SubscriptionManager>();
  rclcpp::QoS sub_qos(rclcpp::QoS{10}.reliability(rclcpp::ReliabilityPolicy::Reliable));
  // Create a subscription to the messages_lost event
  sub->add_subscription<rosbag2_interfaces::msg::MessagesLostEvent>(
    topic_name, expected_number_of_messages, sub_qos
  );
  // Wait for discovery to match publishers with subscribers
  ASSERT_TRUE(sub->spin_and_wait_for_matched({topic_name}, std::chrono::seconds(30), 1));
  auto await_received_messages = sub->spin_subscriptions(std::chrono::seconds(30));

  const std::vector<rosbag2_cpp::bag_events::MessagesLostInfo> msgs_lost_in_recorder = {
    {"topic1", 3u},
    {"topic3", 9u}
  };
  notifier_->on_messages_lost_in_recorder(msgs_lost_in_recorder);

  await_received_messages.get();
  auto received_msgs_lost_event_messages =
    sub->get_received_messages<rosbag2_interfaces::msg::MessagesLostEvent>(topic_name);
  ASSERT_THAT(received_msgs_lost_event_messages, SizeIs(expected_number_of_messages));

  EXPECT_THAT(received_msgs_lost_event_messages[0]->node_name,
              Eq(node_->get_fully_qualified_name()));

  const auto & msgs_lost_statistics =
    received_msgs_lost_event_messages[0]->messages_lost_statistics;
  ASSERT_THAT(msgs_lost_statistics.size(), Eq(2u));

  EXPECT_THAT(
    msgs_lost_statistics,
    Contains(
      AllOf(
        Field(&MessagesLostEventTopicStat::topic_name, Eq(msgs_lost_in_recorder[0].topic_name)),
        Field(&MessagesLostEventTopicStat::messages_lost_in_recorder,
              Eq(msgs_lost_in_recorder[0].num_messages_lost)),
        Field(&MessagesLostEventTopicStat::messages_lost_in_transport, Eq(0u))
      )
    )
  );

  EXPECT_THAT(
    msgs_lost_statistics,
    Contains(
      AllOf(
        Field(&MessagesLostEventTopicStat::topic_name, Eq(msgs_lost_in_recorder[1].topic_name)),
        Field(&MessagesLostEventTopicStat::messages_lost_in_recorder,
              Eq(msgs_lost_in_recorder[1].num_messages_lost)),
        Field(&MessagesLostEventTopicStat::messages_lost_in_transport, Eq(0u))
      )
    )
  );
}

TEST_F(TestRecorderEventNotifier, can_publish_event_on_messages_lost_in_transport)
{
  using MessagesLostEventTopicStat = rosbag2_interfaces::msg::MessagesLostEventTopicStat;
  notifier_->set_messages_lost_statistics_max_publishing_rate(30.0f);
  const size_t expected_number_of_messages = 1;
  const std::string topic_name = RecorderEventNotifier::get_default_messages_lost_topic_name();
  auto sub = std::make_unique<SubscriptionManager>();
  rclcpp::QoS sub_qos(rclcpp::QoS{10}.reliability(rclcpp::ReliabilityPolicy::Reliable));
  // Create a subscription to the messages_lost event
  sub->add_subscription<rosbag2_interfaces::msg::MessagesLostEvent>(
    topic_name, expected_number_of_messages, sub_qos
  );
  // Wait for discovery to match publishers with subscribers
  ASSERT_TRUE(sub->spin_and_wait_for_matched({topic_name}, std::chrono::seconds(30), 1));
  auto await_received_messages = sub->spin_subscriptions(std::chrono::seconds(30));

  rclcpp::QOSMessageLostInfo msgs_lost_in_transport;
  msgs_lost_in_transport.total_count = 20;
  msgs_lost_in_transport.total_count_change = 5;
  notifier_->on_messages_lost_in_transport("topic1", msgs_lost_in_transport);

  await_received_messages.get();
  auto received_msgs_lost_event_messages =
    sub->get_received_messages<rosbag2_interfaces::msg::MessagesLostEvent>(topic_name);
  ASSERT_THAT(received_msgs_lost_event_messages, SizeIs(expected_number_of_messages));

  EXPECT_THAT(received_msgs_lost_event_messages[0]->node_name,
              Eq(node_->get_fully_qualified_name()));

  const auto & msgs_lost_statistics =
    received_msgs_lost_event_messages[0]->messages_lost_statistics;
  ASSERT_THAT(msgs_lost_statistics.size(), Eq(1u));

  EXPECT_THAT(
    msgs_lost_statistics,
    Contains(
      AllOf(
        Field(&MessagesLostEventTopicStat::topic_name, Eq("topic1")),
        Field(&MessagesLostEventTopicStat::messages_lost_in_recorder, Eq(0u)),
        Field(&MessagesLostEventTopicStat::messages_lost_in_transport,
              Eq(msgs_lost_in_transport.total_count_change))
      )
    )
  );
}

TEST_F(TestRecorderEventNotifier, not_publishing_on_messages_lost_event_when_disabled)
{
  notifier_->set_messages_lost_statistics_max_publishing_rate(0.0f);  // Disable publishing
  const size_t expected_number_of_messages = 0;
  const std::string topic_name = RecorderEventNotifier::get_default_messages_lost_topic_name();
  auto sub = std::make_unique<SubscriptionManager>();
  rclcpp::QoS sub_qos(rclcpp::QoS{10}.reliability(rclcpp::ReliabilityPolicy::Reliable));
  // Create a subscription to the messages_lost event
  sub->add_subscription<rosbag2_interfaces::msg::MessagesLostEvent>(topic_name, 1, sub_qos);

  // Wait for discovery to match publishers with subscribers
  ASSERT_TRUE(sub->spin_and_wait_for_matched({topic_name}, std::chrono::seconds(30), 1));
  auto await_received_messages = sub->spin_subscriptions(std::chrono::seconds(3));

  const std::vector<rosbag2_cpp::bag_events::MessagesLostInfo> msgs_lost_in_recorder = {
    {"topic1", 3u},
    {"topic3", 9u}
  };
  notifier_->on_messages_lost_in_recorder(msgs_lost_in_recorder);

  // Check that no messages lost event is published even if the other events are triggered
  rosbag2_cpp::bag_events::BagSplitInfo bag_split_info1;
  bag_split_info1.closed_file = "closed_file1.bag";
  bag_split_info1.opened_file = "";
  notifier_->on_bag_split_in_recorder(bag_split_info1);

  await_received_messages.get();
  auto received_msgs_lost_event_messages =
    sub->get_received_messages<rosbag2_interfaces::msg::MessagesLostEvent>(topic_name);
  ASSERT_THAT(received_msgs_lost_event_messages, SizeIs(expected_number_of_messages));
}

TEST_F(TestRecorderEventNotifier, will_publish_messages_lost_event_when_updating_pub_rate)
{
  using MessagesLostEventTopicStat = rosbag2_interfaces::msg::MessagesLostEventTopicStat;
  // Disable publishing at the beginning
  notifier_->set_messages_lost_statistics_max_publishing_rate(0.0f);
  const size_t expected_number_of_messages = 1;
  const std::string topic_name = RecorderEventNotifier::get_default_messages_lost_topic_name();
  auto sub = std::make_unique<SubscriptionManager>();
  rclcpp::QoS sub_qos(rclcpp::QoS{10}.reliability(rclcpp::ReliabilityPolicy::Reliable));
  // Create a subscription to the messages_lost event
  sub->add_subscription<rosbag2_interfaces::msg::MessagesLostEvent>(
    topic_name, expected_number_of_messages, sub_qos
  );
  // Wait for discovery to match publishers with subscribers
  ASSERT_TRUE(sub->spin_and_wait_for_matched({topic_name}, std::chrono::seconds(30), 1));
  auto await_received_messages = sub->spin_subscriptions(std::chrono::seconds(30));

  const std::vector<rosbag2_cpp::bag_events::MessagesLostInfo> msgs_lost_in_recorder = {
    {"topic1", 3u},
    {"topic2", 9u}
  };
  notifier_->on_messages_lost_in_recorder(msgs_lost_in_recorder);

  rclcpp::QOSMessageLostInfo msgs_lost_in_transport;
  msgs_lost_in_transport.total_count = 20;
  msgs_lost_in_transport.total_count_change = 5;
  notifier_->on_messages_lost_in_transport("topic3", msgs_lost_in_transport);

  // Now enable publishing of messages lost statistics
  notifier_->set_messages_lost_statistics_max_publishing_rate(30.0f);

  await_received_messages.get();
  auto received_msgs_lost_event_messages =
    sub->get_received_messages<rosbag2_interfaces::msg::MessagesLostEvent>(topic_name);
  ASSERT_THAT(received_msgs_lost_event_messages, SizeIs(expected_number_of_messages));

  EXPECT_THAT(received_msgs_lost_event_messages[0]->node_name,
              Eq(node_->get_fully_qualified_name()));

  const auto & msgs_lost_statistics =
    received_msgs_lost_event_messages[0]->messages_lost_statistics;
  ASSERT_THAT(msgs_lost_statistics.size(), Eq(3u));

  EXPECT_THAT(
    msgs_lost_statistics,
    Contains(
      AllOf(
        Field(&MessagesLostEventTopicStat::topic_name, Eq(msgs_lost_in_recorder[0].topic_name)),
        Field(&MessagesLostEventTopicStat::messages_lost_in_recorder,
              Eq(msgs_lost_in_recorder[0].num_messages_lost)),
        Field(&MessagesLostEventTopicStat::messages_lost_in_transport, Eq(0u))
      )
    )
  );

  EXPECT_THAT(
    msgs_lost_statistics,
    Contains(
      AllOf(
        Field(&MessagesLostEventTopicStat::topic_name, Eq(msgs_lost_in_recorder[1].topic_name)),
        Field(&MessagesLostEventTopicStat::messages_lost_in_recorder,
              Eq(msgs_lost_in_recorder[1].num_messages_lost)),
        Field(&MessagesLostEventTopicStat::messages_lost_in_transport, Eq(0u))
      )
    )
  );

  EXPECT_THAT(
    msgs_lost_statistics,
    Contains(
      AllOf(
        Field(&MessagesLostEventTopicStat::topic_name, Eq("topic3")),
        Field(&MessagesLostEventTopicStat::messages_lost_in_recorder, Eq(0u)),
        Field(&MessagesLostEventTopicStat::messages_lost_in_transport,
              Eq(msgs_lost_in_transport.total_count_change))
      )
    )
  );
}

TEST_F(TestRecorderEventNotifier, event_notifier_respects_max_publishing_rate) {
  std::vector<WriteSplitEvent> published_write_split_events;
  std::vector<std::pair<std::chrono::steady_clock::time_point, MessagesLostEvent>>
    published_messages_lost_events;
  std::mutex published_events_mutex;

  // Create shared_ptrs to the mock wrappers first
  auto write_split_pub_mock = std::make_shared<MockPublisherWrapper<WriteSplitEvent>>(
    node_->create_publisher<WriteSplitEvent>(
      RecorderEventNotifier::get_default_write_split_topic_name(), 1));

  auto msgs_lost_pub_mock = std::make_shared<MockPublisherWrapper<MessagesLostEvent>>(
    node_->create_publisher<MessagesLostEvent>(
      RecorderEventNotifier::get_default_messages_lost_topic_name(), 1));

  // Set up expectations on the shared_ptr mocks directly
  ON_CALL(*write_split_pub_mock, publish(::testing::_))
  .WillByDefault(::testing::Invoke(
      [&published_write_split_events, &published_events_mutex](const WriteSplitEvent & msg) {
        std::lock_guard<std::mutex> lock(published_events_mutex);
        published_write_split_events.push_back(msg);
      }
  ));

  EXPECT_CALL(*msgs_lost_pub_mock, publish(::testing::_))
  .WillRepeatedly(::testing::Invoke(
      [&published_messages_lost_events, &published_events_mutex](const MessagesLostEvent & msg) {
        std::lock_guard<std::mutex> lock(published_events_mutex);
        published_messages_lost_events.emplace_back(std::chrono::steady_clock::now(), msg);
      }
  ));

  // Pass the shared_ptr mocks directly to the notifier
  notifier_ =
    std::make_unique<RecorderEventNotifier>(node_.get(), write_split_pub_mock, msgs_lost_pub_mock);

  notifier_->set_messages_lost_statistics_max_publishing_rate(2.0f);  // 2 Hz

  // Trigger multiple messages lost events
  rclcpp::QOSMessageLostInfo qos_msgs_lost_info;
  qos_msgs_lost_info.total_count_change = 1;

  // Trigger the event 10 times with delay in 100 ms interval. i.e., 10 events in 1 second
  for (int i = 0; i < 10; ++i) {
    notifier_->on_messages_lost_in_transport("topic1", qos_msgs_lost_info);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Simulate time between events
  }

  // Allow some time for the events to be processed
  std::this_thread::sleep_for(std::chrono::seconds(1));
  notifier_.reset();  // Ensure all events are processed before assertions

  std::lock_guard<std::mutex> lock(published_events_mutex);
  // Verify that no write split events were published
  EXPECT_TRUE(published_write_split_events.empty());

  // Verify that the number of published messages respects the 2 Hz rate
  ASSERT_GE(published_messages_lost_events.size(), 2u);  // At least 2 events in 1 second
  ASSERT_LE(published_messages_lost_events.size(), 3u);  // At most 3 events in 1 second

  // Verify the time intervals between published events
  for (size_t i = 1; i < published_messages_lost_events.size(); ++i) {
    auto time_diff = std::chrono::duration_cast<std::chrono::milliseconds>(
      published_messages_lost_events[i].first - published_messages_lost_events[i - 1].first);
    EXPECT_GE(time_diff.count(), 500);  // At least 500 ms between events (2 Hz)
  }
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

TEST_F(TestRecorderEventNotifier, messages_lost_in_recorder_correctly_accumulated)
{
  std::vector<rosbag2_cpp::bag_events::MessagesLostInfo> msgs_lost_info = {
    {"topic1", 3},
    {"topic3", 9}
  };
  notifier_->on_messages_lost_in_recorder(msgs_lost_info);
  notifier_->on_messages_lost_in_recorder(msgs_lost_info);

  EXPECT_EQ(notifier_->get_total_num_messages_lost_in_recorder(), 24u);
}

TEST_F(TestRecorderEventNotifier, reset_messages_lost_counters)
{
  std::vector<rosbag2_cpp::bag_events::MessagesLostInfo> msgs_lost_info = {
    {"topic1", 5}
  };
  notifier_->on_messages_lost_in_recorder(msgs_lost_info);

  rclcpp::QOSMessageLostInfo qos_msgs_lost_info;
  qos_msgs_lost_info.total_count_change = 3;
  ASSERT_NO_THROW(notifier_->on_messages_lost_in_transport("topic2", qos_msgs_lost_info));

  EXPECT_EQ(notifier_->get_total_num_messages_lost_in_recorder(), 5u);
  EXPECT_EQ(notifier_->get_total_num_messages_lost_in_transport(), 3u);

  notifier_->reset_total_num_messages_lost_in_recorder();
  notifier_->reset_total_num_messages_lost_in_transport();

  EXPECT_EQ(notifier_->get_total_num_messages_lost_in_recorder(), 0u);
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

TEST_F(TestRecorderEventNotifier, handle_empty_messages_lost_in_recorder)
{
  std::vector<rosbag2_cpp::bag_events::MessagesLostInfo> msgs_lost_info = {};
  ASSERT_NO_THROW(notifier_->on_messages_lost_in_recorder(msgs_lost_info));
  EXPECT_EQ(notifier_->get_total_num_messages_lost_in_recorder(), 0u);
}

TEST_F(TestRecorderEventNotifier, zero_messages_lost)
{
  rclcpp::QOSMessageLostInfo qos_msgs_lost_info;
  qos_msgs_lost_info.total_count = 0;
  qos_msgs_lost_info.total_count_change = 0;
  ASSERT_NO_THROW(notifier_->on_messages_lost_in_transport("topic1", qos_msgs_lost_info));

  std::vector<rosbag2_cpp::bag_events::MessagesLostInfo> msgs_lost_info = {
    {"topic1", 0}
  };
  ASSERT_NO_THROW(notifier_->on_messages_lost_in_recorder(msgs_lost_info));

  EXPECT_EQ(notifier_->get_total_num_messages_lost_in_transport(), 0u);
  EXPECT_EQ(notifier_->get_total_num_messages_lost_in_recorder(), 0u);
}

TEST_F(TestRecorderEventNotifier, large_message_counts)
{
  rclcpp::QOSMessageLostInfo qos_msgs_lost_info;
  qos_msgs_lost_info.total_count = 1000000;
  qos_msgs_lost_info.total_count_change = 1000000;
  notifier_->on_messages_lost_in_transport("topic1", qos_msgs_lost_info);

  std::vector<rosbag2_cpp::bag_events::MessagesLostInfo> msgs_lost_info = {
    {"topic1", 2000000}
  };
  notifier_->on_messages_lost_in_recorder(msgs_lost_info);

  EXPECT_EQ(notifier_->get_total_num_messages_lost_in_transport(), 1000000u);
  EXPECT_EQ(notifier_->get_total_num_messages_lost_in_recorder(), 2000000u);
}

TEST_F(TestRecorderEventNotifier, reset_between_message_loss_notifications)
{
  rclcpp::QOSMessageLostInfo qos_msgs_lost_info;
  qos_msgs_lost_info.total_count = 1000000;
  qos_msgs_lost_info.total_count_change = 10;
  notifier_->on_messages_lost_in_transport("topic1", qos_msgs_lost_info);

  std::vector<rosbag2_cpp::bag_events::MessagesLostInfo> msgs_lost_info = {
    {"topic1", 5}
  };
  notifier_->on_messages_lost_in_recorder(msgs_lost_info);

  notifier_->reset_total_num_messages_lost_in_transport();
  notifier_->reset_total_num_messages_lost_in_recorder();

  qos_msgs_lost_info.total_count_change = 7;
  notifier_->on_messages_lost_in_transport("topic2", qos_msgs_lost_info);

  msgs_lost_info = {{"topic2", 3}};
  notifier_->on_messages_lost_in_recorder(msgs_lost_info);

  EXPECT_EQ(notifier_->get_total_num_messages_lost_in_transport(), 7u);
  EXPECT_EQ(notifier_->get_total_num_messages_lost_in_recorder(), 3u);
}

TEST_F(TestRecorderEventNotifier, thread_safety_with_concurrent_access)
{
  constexpr size_t num_threads_for_each_event = 10;
  constexpr size_t iterations_per_thread = 1000;
  constexpr size_t expected_transport_lost = num_threads_for_each_event * iterations_per_thread;
  constexpr size_t expected_recorder_lost = num_threads_for_each_event * iterations_per_thread;

  std::vector<std::thread> threads;

  // Simulate concurrent access to on_messages_lost_in_transport
  for (size_t i = 0; i < num_threads_for_each_event; i++) {
    // Simulate concurrent access to on_messages_lost_in_transport
    threads.emplace_back([this, i]() {
        for (size_t j = 0; j < iterations_per_thread; j++) {
          rclcpp::QOSMessageLostInfo qos_msgs_lost_info;
          qos_msgs_lost_info.total_count_change = 1;
          notifier_->on_messages_lost_in_transport("topic" + std::to_string(i), qos_msgs_lost_info);
        }
    });
    // Simulate concurrent access to on_messages_lost_in_recorder
    threads.emplace_back([this, i]() {
        for (size_t j = 0; j < iterations_per_thread; j++) {
          std::vector<rosbag2_cpp::bag_events::MessagesLostInfo> msgs_lost_info = {
            {"topic" + std::to_string(i), 1}
          };
          notifier_->on_messages_lost_in_recorder(msgs_lost_info);
        }
    });
  }

  for (auto & thread : threads) {
    thread.join();
  }

  // Verify that the total counts are consistent
  EXPECT_EQ(notifier_->get_total_num_messages_lost_in_transport(), expected_transport_lost);
  EXPECT_EQ(notifier_->get_total_num_messages_lost_in_recorder(), expected_recorder_lost);
}
