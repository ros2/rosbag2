// Copyright 2021, Apex.AI
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

#include <chrono>
#include <future>
#include <memory>
#include <utility>
#include <vector>

#include "mock_player.hpp"
#include "rosbag2_play_test_fixture.hpp"
#include "test_msgs/message_fixtures.hpp"
#include "test_msgs/msg/arrays.hpp"
#include "test_msgs/msg/basic_types.hpp"
#include "test_msgs/srv/basic_types.hpp"

using namespace ::testing;  // NOLINT
using namespace rosbag2_transport;  // NOLINT
using namespace rosbag2_test_common;  // NOLINT
using namespace std::chrono_literals;  // NOLINT

namespace
{
static inline std::vector<test_msgs::srv::BasicTypes_Event::SharedPtr>
get_service_event_message_basic_types()
{
  std::vector<test_msgs::srv::BasicTypes_Event::SharedPtr> messages;

  {
    auto msg = std::make_shared<test_msgs::srv::BasicTypes_Event>();
    msg->info.event_type = service_msgs::msg::ServiceEventInfo::REQUEST_RECEIVED;
    test_msgs::srv::BasicTypes_Request request;
    request.int32_value = 123;
    request.int64_value = 456;
    msg->request.emplace_back(request);
    messages.push_back(msg);
  }

  {
    auto msg = std::make_shared<test_msgs::srv::BasicTypes_Event>();
    msg->info.event_type = service_msgs::msg::ServiceEventInfo::REQUEST_RECEIVED;
    test_msgs::srv::BasicTypes_Request request;
    request.int32_value = 456;
    request.int64_value = 789;
    msg->request.emplace_back(request);
    messages.push_back(msg);
  }

  return messages;
}
}  // namespace

TEST_F(RosBag2PlayTestFixture, burst_with_false_preconditions) {
  auto primitive_message = get_messages_basic_types()[0];
  primitive_message->int32_value = 42;

  auto topic_types = std::vector<rosbag2_storage::TopicMetadata>{
    {1u, "topic1", "test_msgs/BasicTypes", "", {}, ""}};

  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages =
  {serialize_test_message("topic1", 2100, primitive_message)};

  auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
  prepared_mock_reader->prepare(messages, topic_types);
  auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));
  auto player = std::make_shared<MockPlayer>(std::move(reader), storage_options_, play_options_);

  ASSERT_FALSE(player->is_paused());
  ASSERT_EQ(player->burst(1), 0u);
  player->pause();
  ASSERT_TRUE(player->is_paused());
}

TEST_F(RosBag2PlayTestFixture, burst_bursts_requested_messages_without_delays) {
  auto primitive_message = get_messages_basic_types()[0];
  primitive_message->int32_value = 42;

  auto topic_types = std::vector<rosbag2_storage::TopicMetadata>{
    {1u, "topic1", "test_msgs/BasicTypes", "", {}, ""}};

  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages =
  {
    serialize_test_message("topic1", 2100, primitive_message),
    serialize_test_message("topic1", 3300, primitive_message),
    serialize_test_message("topic1", 4600, primitive_message),
    serialize_test_message("topic1", 5900, primitive_message)
  };

  auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
  prepared_mock_reader->prepare(messages, topic_types);
  auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));
  auto player = std::make_shared<MockPlayer>(std::move(reader), storage_options_, play_options_);

  sub_ = std::make_shared<SubscriptionManager>();
  sub_->add_subscription<test_msgs::msg::BasicTypes>("/topic1", messages.size());

  // Wait for discovery to match publishers with subscribers
  ASSERT_TRUE(
    sub_->spin_and_wait_for_matched(player->get_list_of_publishers(), std::chrono::seconds(30)));

  auto await_received_messages = sub_->spin_subscriptions();

  player->pause();
  ASSERT_TRUE(player->is_paused());

  player->play();
  player->wait_for_playback_to_start();

  const size_t NUM_MESSAGES_TO_BURST = 4;

  ASSERT_TRUE(player->is_paused());
  auto start = std::chrono::steady_clock::now();
  size_t played_messages = player->burst(NUM_MESSAGES_TO_BURST);
  auto replay_time = std::chrono::steady_clock::now() - start;
  ASSERT_EQ(played_messages, NUM_MESSAGES_TO_BURST);
  ASSERT_THAT(replay_time, Lt(std::chrono::seconds(2)));

  ASSERT_TRUE(player->is_paused());
  player->resume();
  player->wait_for_playback_to_finish();

  await_received_messages.get();

  auto replayed_topic1 = sub_->get_received_messages<test_msgs::msg::BasicTypes>("/topic1");
  EXPECT_THAT(replayed_topic1, SizeIs(NUM_MESSAGES_TO_BURST));
}

TEST_F(RosBag2PlayTestFixture, burst_stops_at_end_of_file) {
  auto primitive_message = get_messages_basic_types()[0];
  primitive_message->int32_value = 42;

  auto topic_types = std::vector<rosbag2_storage::TopicMetadata>{
    {1u, "topic1", "test_msgs/BasicTypes", "", {}, ""}};

  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages =
  {
    serialize_test_message("topic1", 2100, primitive_message),
    serialize_test_message("topic1", 3300, primitive_message),
    serialize_test_message("topic1", 4600, primitive_message),
    serialize_test_message("topic1", 5900, primitive_message)
  };

  auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
  prepared_mock_reader->prepare(messages, topic_types);
  auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));
  auto player = std::make_shared<MockPlayer>(std::move(reader), storage_options_, play_options_);

  sub_ = std::make_shared<SubscriptionManager>();
  sub_->add_subscription<test_msgs::msg::BasicTypes>("/topic1", messages.size());

  // Wait for discovery to match publishers with subscribers
  ASSERT_TRUE(
    sub_->spin_and_wait_for_matched(player->get_list_of_publishers(), std::chrono::seconds(30)));

  auto await_received_messages = sub_->spin_subscriptions();

  player->pause();
  ASSERT_TRUE(player->is_paused());

  player->play();
  player->wait_for_playback_to_start();

  ASSERT_TRUE(player->is_paused());
  size_t played_messages = player->burst(messages.size() + 1);  // Request one more than available
  ASSERT_EQ(played_messages, messages.size());
  ASSERT_TRUE(player->is_paused());
  player->resume();
  player->wait_for_playback_to_finish();
  await_received_messages.get();

  auto replayed_topic1 = sub_->get_received_messages<test_msgs::msg::BasicTypes>("/topic1");
  EXPECT_THAT(replayed_topic1, SizeIs(messages.size()));
}

TEST_F(RosBag2PlayTestFixture, burst_bursting_one_by_one_messages_with_the_same_timestamp) {
  auto primitive_message = get_messages_basic_types()[0];
  primitive_message->int32_value = 42;

  auto topic_types = std::vector<rosbag2_storage::TopicMetadata>{
    {1u, "topic1", "test_msgs/BasicTypes", "", {}, ""}};

  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages =
  {
    serialize_test_message("topic1", 1000, primitive_message),
    serialize_test_message("topic1", 1000, primitive_message),
    serialize_test_message("topic1", 1000, primitive_message),
    serialize_test_message("topic1", 1000, primitive_message)
  };

  auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
  prepared_mock_reader->prepare(messages, topic_types);
  auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));
  auto player = std::make_shared<MockPlayer>(std::move(reader), storage_options_, play_options_);

  sub_ = std::make_shared<SubscriptionManager>();
  sub_->add_subscription<test_msgs::msg::BasicTypes>("/topic1", messages.size());

  // Wait for discovery to match publishers with subscribers
  ASSERT_TRUE(
    sub_->spin_and_wait_for_matched(player->get_list_of_publishers(), std::chrono::seconds(30)));

  auto await_received_messages = sub_->spin_subscriptions();

  player->pause();
  ASSERT_TRUE(player->is_paused());

  player->play();

  ASSERT_TRUE(player->is_paused());
  ASSERT_EQ(player->burst(1), 1u);
  size_t played_messages = 1;
  while (player->burst(1) == 1) {
    // Yield CPU resources for player-play() running in separate thread to make sure that it
    // will not play extra messages.
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    played_messages++;
  }
  ASSERT_EQ(played_messages, messages.size());
  ASSERT_TRUE(player->is_paused());
  player->resume();
  player->wait_for_playback_to_finish();
  await_received_messages.get();

  auto replayed_topic1 = sub_->get_received_messages<test_msgs::msg::BasicTypes>("/topic1");
  EXPECT_THAT(replayed_topic1, SizeIs(messages.size()));
}

TEST_F(RosBag2PlayTestFixture, play_respect_messages_timing_after_burst) {
  auto primitive_message = get_messages_basic_types()[0];
  primitive_message->int32_value = 42;

  auto topic_types = std::vector<rosbag2_storage::TopicMetadata>{
    {1u, "topic1", "test_msgs/BasicTypes", "", {}, ""}};

  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages =
  {
    serialize_test_message("topic1", 1500, primitive_message),
    serialize_test_message("topic1", 2500, primitive_message),
    serialize_test_message("topic1", 2700, primitive_message),
    serialize_test_message("topic1", 2800, primitive_message)
  };

  auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
  prepared_mock_reader->prepare(messages, topic_types);
  auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));
  auto player = std::make_shared<MockPlayer>(std::move(reader), storage_options_, play_options_);

  sub_ = std::make_shared<SubscriptionManager>();
  sub_->add_subscription<test_msgs::msg::BasicTypes>("/topic1", messages.size());

  // Wait for discovery to match publishers with subscribers
  ASSERT_TRUE(
    sub_->spin_and_wait_for_matched(player->get_list_of_publishers(), std::chrono::seconds(30)));

  auto await_received_messages = sub_->spin_subscriptions();

  player->pause();
  ASSERT_TRUE(player->is_paused());

  player->play();

  const size_t EXPECTED_BURST_COUNT = 2;
  ASSERT_TRUE(player->is_paused());
  ASSERT_EQ(player->burst(EXPECTED_BURST_COUNT), EXPECTED_BURST_COUNT);
  ASSERT_TRUE(player->is_paused());
  player->resume();
  auto start = std::chrono::steady_clock::now();
  player->wait_for_playback_to_finish();
  auto replay_time = std::chrono::steady_clock::now() - start;

  auto expected_replay_time =
    std::chrono::nanoseconds(messages.back()->recv_timestamp - messages[1]->recv_timestamp);
  // Check for lower bound with some tolerance
  ASSERT_THAT(replay_time, Gt(expected_replay_time - std::chrono::milliseconds(50)));
  // Check for upper bound with some tolerance
  ASSERT_THAT(replay_time, Lt(expected_replay_time + std::chrono::milliseconds(100)));

  await_received_messages.get();
  auto replayed_topic1 = sub_->get_received_messages<test_msgs::msg::BasicTypes>("/topic1");
  EXPECT_THAT(replayed_topic1, SizeIs(messages.size()));
}

TEST_F(RosBag2PlayTestFixture, player_can_resume_after_burst) {
  auto primitive_message = get_messages_basic_types()[0];
  primitive_message->int32_value = 42;

  auto topic_types = std::vector<rosbag2_storage::TopicMetadata>{
    {1u, "topic1", "test_msgs/BasicTypes", "", {}, ""}};

  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages =
  {
    serialize_test_message("topic1", 300, primitive_message),
    serialize_test_message("topic1", 500, primitive_message),
    serialize_test_message("topic1", 700, primitive_message)
  };

  auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
  prepared_mock_reader->prepare(messages, topic_types);
  auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));
  auto player = std::make_shared<MockPlayer>(std::move(reader), storage_options_, play_options_);

  sub_ = std::make_shared<SubscriptionManager>();
  sub_->add_subscription<test_msgs::msg::BasicTypes>("/topic1", messages.size());

  // Wait for discovery to match publishers with subscribers
  ASSERT_TRUE(
    sub_->spin_and_wait_for_matched(player->get_list_of_publishers(), std::chrono::seconds(30)));

  auto await_received_messages = sub_->spin_subscriptions();

  player->pause();
  ASSERT_TRUE(player->is_paused());

  player->play();

  ASSERT_TRUE(player->is_paused());
  ASSERT_EQ(player->burst(1), 1u);
  ASSERT_TRUE(player->is_paused());
  player->resume();
  player->wait_for_playback_to_finish();
  await_received_messages.get();

  auto replayed_topic1 = sub_->get_received_messages<test_msgs::msg::BasicTypes>("/topic1");
  EXPECT_THAT(replayed_topic1, SizeIs(messages.size()));
}

TEST_F(RosBag2PlayTestFixture, burst_bursting_only_filtered_topics) {
  auto primitive_message1 = get_messages_basic_types()[0];
  primitive_message1->int32_value = 42;

  auto complex_message1 = get_messages_arrays()[0];
  complex_message1->float32_values = {{40.0f, 2.0f, 0.0f}};
  complex_message1->bool_values = {{true, false, true}};

  auto topic_types = std::vector<rosbag2_storage::TopicMetadata>{
    {1u, "/topic1", "test_msgs/BasicTypes", "", {}, ""},
    {2u, "/topic2", "test_msgs/Arrays", "", {}, ""},
  };

  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages =
  {
    serialize_test_message("/topic1", 500, primitive_message1),
    serialize_test_message("/topic1", 700, primitive_message1),
    serialize_test_message("/topic1", 900, primitive_message1),
    serialize_test_message("/topic2", 550, complex_message1),
    serialize_test_message("/topic2", 750, complex_message1),
    serialize_test_message("/topic2", 950, complex_message1)
  };

  // Filter allows /topic2, blocks /topic1
  play_options_.topics_to_filter = {"topic2"};
  auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
  prepared_mock_reader->prepare(messages, topic_types);
  auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));
  auto player = std::make_shared<MockPlayer>(std::move(reader), storage_options_, play_options_);

  sub_ = std::make_shared<SubscriptionManager>();
  sub_->add_subscription<test_msgs::msg::BasicTypes>("/topic1", 0);
  sub_->add_subscription<test_msgs::msg::Arrays>("/topic2", 3);

  // Wait for discovery to match publishers with subscribers
  ASSERT_TRUE(
    sub_->spin_and_wait_for_matched(player->get_list_of_publishers(), std::chrono::seconds(30)));

  auto await_received_messages = sub_->spin_subscriptions();

  player->pause();
  ASSERT_TRUE(player->is_paused());

  player->play();
  ASSERT_TRUE(player->is_paused());

  const size_t EXPECTED_BURST_COUNT = 3;
  ASSERT_EQ(player->burst(EXPECTED_BURST_COUNT), EXPECTED_BURST_COUNT);

  ASSERT_TRUE(player->is_paused());
  player->resume();
  player->wait_for_playback_to_finish();
  await_received_messages.get();

  auto replayed_topic1 = sub_->get_received_messages<test_msgs::msg::BasicTypes>("/topic1");
  // No messages are allowed to have arrived
  EXPECT_THAT(replayed_topic1, SizeIs(0u));

  auto replayed_topic2 = sub_->get_received_messages<test_msgs::msg::Arrays>("/topic2");
  // All we care is that any messages arrived
  EXPECT_THAT(replayed_topic2, SizeIs(Eq(EXPECTED_BURST_COUNT)));
}

TEST_F(RosBag2PlayTestFixture, burst_bursting_only_filtered_services) {
  if (std::string(rmw_get_implementation_identifier()).find("rmw_connextdds") !=
    std::string::npos)
  {
    GTEST_SKIP() << "Skipping. The test is know to be flaky on the rmw_connextdds.";
  }
  const std::string service_name1 = "/test_service1";
  const std::string service_name2 = "/test_service2";
  const std::string service_event_name1 = service_name1 + "/_service_event";
  const std::string service_event_name2 = service_name2 + "/_service_event";

  auto services_types = std::vector<rosbag2_storage::TopicMetadata>{
    {1u, service_event_name1, "test_msgs/srv/BasicTypes_Event", "", {}, ""},
    {2u, service_event_name2, "test_msgs/srv/BasicTypes_Event", "", {}, ""},
  };
  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages =
  {
    serialize_test_message(service_event_name1, 500, get_service_event_message_basic_types()[0]),
    serialize_test_message(service_event_name2, 600, get_service_event_message_basic_types()[0]),
    serialize_test_message(service_event_name1, 400, get_service_event_message_basic_types()[1]),
    serialize_test_message(service_event_name2, 500, get_service_event_message_basic_types()[1])
  };

  std::vector<std::shared_ptr<test_msgs::srv::BasicTypes::Request>> service1_receive_requests;
  std::vector<std::shared_ptr<test_msgs::srv::BasicTypes::Request>> service2_receive_requests;

  srv_->setup_service<test_msgs::srv::BasicTypes>(service_name1, service1_receive_requests);
  srv_->setup_service<test_msgs::srv::BasicTypes>(service_name2, service2_receive_requests);

  srv_->run_services();

  // Filter allows test_service2, blocks test_service1
  play_options_.services_to_filter = {service_event_name2};
  play_options_.publish_service_requests = true;
  auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
  prepared_mock_reader->prepare(messages, services_types);
  auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));
  auto player = std::make_shared<MockPlayer>(std::move(reader), storage_options_, play_options_);

  player->pause();
  ASSERT_TRUE(player->is_paused());

  // Check services are ready
  ASSERT_TRUE(srv_->all_services_ready());

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(player);
  auto spin_thread = std::thread([&exec]() {exec.spin();});
  player->play();

  const size_t EXPECTED_BURST_COUNT = 2;
  ASSERT_EQ(player->burst(EXPECTED_BURST_COUNT), EXPECTED_BURST_COUNT);

  ASSERT_TRUE(player->is_paused());
  player->resume();

  player->wait_for_playback_to_finish();
  EXPECT_TRUE(player->wait_for_sent_service_requests_to_finish("", 2s));

  exec.cancel();
  spin_thread.join();

  EXPECT_EQ(service1_receive_requests.size(), 0);
  EXPECT_EQ(service2_receive_requests.size(), 2);
}
