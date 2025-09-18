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

#include <gmock/gmock.h>

#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <utility>

#include "rcutils/time.h"
#include "rclcpp/rclcpp.hpp"

#include "rosbag2_test_common/subscription_manager.hpp"
#include "rosbag2_test_common/service_manager.hpp"

#include "rosbag2_transport/player.hpp"

#include "test_msgs/msg/arrays.hpp"
#include "test_msgs/msg/basic_types.hpp"
#include "test_msgs/message_fixtures.hpp"
#include "test_msgs/srv/basic_types.hpp"

#include "rosbag2_storage/qos.hpp"

#include "rosbag2_play_test_fixture.hpp"
#include "rosbag2_transport_test_fixture.hpp"
#include "mock_player.hpp"

using namespace ::testing;  // NOLINT
using namespace rosbag2_transport;  // NOLINT
using namespace std::chrono_literals;  // NOLINT
using namespace rosbag2_test_common;  // NOLINT

namespace
{
inline std::vector<test_msgs::srv::BasicTypes_Event::SharedPtr>
get_service_event_message_basic_types()
{
  std::vector<test_msgs::srv::BasicTypes_Event::SharedPtr> messages;

  {
    auto msg = std::make_shared<test_msgs::srv::BasicTypes_Event>();
    msg->info.event_type = service_msgs::msg::ServiceEventInfo::REQUEST_RECEIVED;
    test_msgs::srv::BasicTypes_Request request;
    request.int32_value = 123;
    request.int64_value = 456;
    request.string_value = "event_type=REQUEST_RECEIVED";
    msg->request.emplace_back(request);
    messages.push_back(msg);
  }

  {
    auto msg = std::make_shared<test_msgs::srv::BasicTypes_Event>();
    msg->info.event_type = service_msgs::msg::ServiceEventInfo::REQUEST_RECEIVED;
    test_msgs::srv::BasicTypes_Request request;
    request.int32_value = 456;
    request.int64_value = 789;
    request.string_value = "event_type=REQUEST_RECEIVED";
    msg->request.emplace_back(request);
    messages.push_back(msg);
  }

  {
    auto msg = std::make_shared<test_msgs::srv::BasicTypes_Event>();
    msg->info.event_type = service_msgs::msg::ServiceEventInfo::REQUEST_SENT;
    test_msgs::srv::BasicTypes_Request request;
    request.int32_value = 789;
    request.int64_value = 123;
    request.string_value = "event_type=REQUEST_SENT";
    msg->request.emplace_back(request);
    messages.push_back(msg);
  }

  return messages;
}

void spin_thread_and_wait_for_sent_service_requests_to_finish(
  std::shared_ptr<rosbag2_transport::Player> player,
  const std::vector<std::string> && service_name_list)
{
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(player);
  auto spin_thread = std::thread(
    [&exec]() {
      exec.spin();
    });
  player->play();
  player->wait_for_playback_to_finish();

  for (const auto & service_name : service_name_list) {
    EXPECT_TRUE(player->wait_for_sent_service_requests_to_finish(service_name, 2s));
  }
  exec.cancel();
  if (spin_thread.joinable()) {spin_thread.join();}
}
}  // namespace

TEST_F(RosBag2PlayTestFixture, recorded_messages_are_played_for_all_topics)
{
  auto primitive_message1 = get_messages_basic_types()[0];
  primitive_message1->int32_value = 42;

  auto complex_message1 = get_messages_arrays()[0];
  complex_message1->float32_values = {{40.0f, 2.0f, 0.0f}};
  complex_message1->bool_values = {{true, false, true}};

  auto topic_types = std::vector<rosbag2_storage::TopicMetadata>{
    {1u, "topic1", "test_msgs/BasicTypes", "", {}, ""},
    {2u, "topic2", "test_msgs/Arrays", "", {}, ""},
  };

  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages =
  {serialize_test_message("topic1", 500, primitive_message1),
    serialize_test_message("topic1", 700, primitive_message1),
    serialize_test_message("topic1", 900, primitive_message1),
    serialize_test_message("topic2", 550, complex_message1),
    serialize_test_message("topic2", 750, complex_message1),
    serialize_test_message("topic2", 950, complex_message1)};

  auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
  prepared_mock_reader->prepare(messages, topic_types);
  auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));

  // Due to a problem related to the subscriber, we play many (3) messages but make the subscriber
  // node spin only until 2 have arrived. Hence the 2 as `launch_subscriber()` argument.
  sub_->add_subscription<test_msgs::msg::BasicTypes>("/topic1", 2);
  sub_->add_subscription<test_msgs::msg::Arrays>("/topic2", 2);

  auto await_received_messages = sub_->spin_subscriptions();

  auto player = std::make_shared<rosbag2_transport::Player>(
    std::move(reader), storage_options_, play_options_);
  player->play();
  player->wait_for_playback_to_finish();
  await_received_messages.get();

  auto replayed_test_primitives = sub_->get_received_messages<test_msgs::msg::BasicTypes>(
    "/topic1");
  EXPECT_THAT(replayed_test_primitives, SizeIs(Ge(2u)));
  EXPECT_THAT(
    replayed_test_primitives,
    Each(Pointee(Field(&test_msgs::msg::BasicTypes::int32_value, 42))));

  auto replayed_test_arrays = sub_->get_received_messages<test_msgs::msg::Arrays>(
    "/topic2");
  EXPECT_THAT(replayed_test_arrays, SizeIs(Ge(2u)));
  EXPECT_THAT(
    replayed_test_arrays,
    Each(
      Pointee(
        Field(
          &test_msgs::msg::Arrays::bool_values,
          ElementsAre(true, false, true)))));
  EXPECT_THAT(
    replayed_test_arrays,
    Each(
      Pointee(
        Field(
          &test_msgs::msg::Arrays::float32_values,
          ElementsAre(40.0f, 2.0f, 0.0f)))));
}

TEST_F(RosBag2PlayTestFixture, can_play_when_one_bag_has_fewer_messages_than_other_bags)
{
  auto msg = get_messages_basic_types()[0];
  msg->int32_value = 42;

  auto topic_types = std::vector<rosbag2_storage::TopicMetadata>{
    {1u, "topic1", "test_msgs/msg/BasicTypes", "", {}, ""},
    {2u, "topic2", "test_msgs/msg/BasicTypes", "", {}, ""},
  };

  std::vector<std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>>> messages_list{};

  messages_list.emplace_back(
    std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>>{
    serialize_test_message("topic1", 1, msg)
    }
  );
  for (size_t i = 0; i < 5; i++) {
    messages_list.back().emplace_back(
      serialize_test_message(i % 2 ? "topic1" : "topic2",
        // Translate nanoseconds to milliseconds before incrementing value
                             messages_list.back().back()->recv_timestamp / 1000000 + 3,
                             msg));
  }

  messages_list.emplace_back(
    std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>>{
    serialize_test_message("topic1", 2, msg)
    }
  );
  // Add more messages to the last bag to make it have more messages than the first one
  for (size_t i = 0; i < 15; i++) {
    messages_list.back().emplace_back(
      serialize_test_message(i % 2 ? "topic1" : "topic2",
        // Translate nanoseconds to milliseconds before incrementing value
                             messages_list.back().back()->recv_timestamp / 1000000 + 3,
                             msg));
  }

  std::vector<rosbag2_transport::Player::reader_storage_options_pair_t> bags{};
  std::size_t total_messages = 0u;
  for (const auto & curr_bag_messages : messages_list) {
    auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
    total_messages += curr_bag_messages.size();
    prepared_mock_reader->prepare(curr_bag_messages, topic_types);
    bags.emplace_back(
      std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader)), storage_options_);
  }
  ASSERT_GT(total_messages, 0u);

  // Note: For the sake of the checking corner cases, the read_ahead_queue_size is set to a value
  // that is larger than the number of messages in the first bag, but smaller than the total number
  // of messages in all bags.
  play_options_.read_ahead_queue_size = 17;
  auto player = std::make_shared<rosbag2_transport::Player>(std::move(bags), play_options_);
  std::size_t num_played_messages = 0u;
  rcutils_time_point_value_t last_timestamp = 0;
  const auto callback = [&](std::shared_ptr<rosbag2_storage::SerializedBagMessage> msg) {
    // Make sure messages are played in order
      const auto timestamp = msg->recv_timestamp;
      EXPECT_LE(last_timestamp, timestamp);
      last_timestamp = timestamp;
      num_played_messages++;
    };
  player->add_on_play_message_pre_callback(callback);
  player->play();
  ASSERT_TRUE(player->wait_for_playback_to_start(10s));
  ASSERT_TRUE(player->wait_for_playback_to_finish(10s));
  EXPECT_EQ(total_messages, num_played_messages);
}

TEST_F(RosBag2PlayTestFixture, recorded_messages_are_played_for_all_topics_from_three_bags)
{
  auto msg = get_messages_basic_types()[0];
  msg->int32_value = 42;

  auto topic_types = std::vector<rosbag2_storage::TopicMetadata>{
    {1u, "topic1", "test_msgs/msg/BasicTypes", "", {}, ""},
    {2u, "topic2", "test_msgs/msg/BasicTypes", "", {}, ""},
  };

  // Make sure each reader's/bag's messages are ordered by time
  // However, do interlace messages across bags
  std::vector<std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>>> messages_list{};
  messages_list.emplace_back(std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>>{
    serialize_test_message("topic1", 1, msg),
    serialize_test_message("topic2", 5, msg),
    serialize_test_message("topic1", 8, msg),
    serialize_test_message("topic2", 10, msg),
    serialize_test_message("topic1", 13, msg),
    serialize_test_message("topic2", 14, msg)});
  messages_list.emplace_back(std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>>{
    serialize_test_message("topic1", 2, msg),
    serialize_test_message("topic2", 3, msg),
    serialize_test_message("topic1", 6, msg),
    serialize_test_message("topic2", 10, msg),
    serialize_test_message("topic1", 12, msg),
    serialize_test_message("topic2", 16, msg)});
  messages_list.emplace_back(std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>>{
    serialize_test_message("topic1", 1, msg),
    serialize_test_message("topic2", 4, msg),
    serialize_test_message("topic1", 7, msg),
    serialize_test_message("topic2", 9, msg),
    serialize_test_message("topic1", 11, msg),
    serialize_test_message("topic2", 15, msg)});
  std::vector<rosbag2_transport::Player::reader_storage_options_pair_t> bags{};
  std::size_t total_messages = 0u;
  for (const auto & curr_bag_messages : messages_list) {
    auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
    total_messages += curr_bag_messages.size();
    prepared_mock_reader->prepare(curr_bag_messages, topic_types);
    bags.emplace_back(
      std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader)), storage_options_);
  }
  ASSERT_GT(total_messages, 0u);

  play_options_.read_ahead_queue_size = total_messages - 3;
  auto player = std::make_shared<rosbag2_transport::Player>(std::move(bags), play_options_);
  std::size_t num_played_messages = 0u;
  rcutils_time_point_value_t last_timetamp = 0;
  const auto callback = [&](std::shared_ptr<rosbag2_storage::SerializedBagMessage> msg) {
      // Make sure messages are played in order
      EXPECT_LE(last_timetamp, msg->recv_timestamp);
      last_timetamp = msg->recv_timestamp;
      num_played_messages++;
    };
  player->add_on_play_message_pre_callback(callback);
  player->play();
  ASSERT_TRUE(player->wait_for_playback_to_start(10s));
  ASSERT_TRUE(player->wait_for_playback_to_finish(10s));
  EXPECT_EQ(total_messages, num_played_messages);
}

TEST_F(RosBag2PlayTestFixture, high_freq_topics_does_not_starve_in_multibag_playback) {
  static constexpr const char * high_freq_topic1_name = "HighFreqTopic1";
  static constexpr const char * low_freq_topic1_name = "LowFreqTopic1";
  static constexpr const char * low_freq_topic2_name = "LowFreqTopic2";
  // High frequency topic is published every 20 ms, low frequency topic every 60 ms
  // Total play time is 1 second, so we expect 50 messages from high frequency topic and 16 from
  // each low frequency topic (if played in isolation).
  constexpr rcutils_duration_value_t high_freq_topic_period_ms = 20;
  constexpr rcutils_duration_value_t low_freq_topic_period_ms = 60;
  constexpr rcutils_duration_value_t total_play_time_ms = 1000;
  const size_t num_high_freq_msgs_per_bag = total_play_time_ms / high_freq_topic_period_ms;
  const size_t num_low_freq_msgs_per_bag = total_play_time_ms / low_freq_topic_period_ms;
  auto msg = get_messages_basic_types()[0];
  msg->int32_value = 42;

  // Each bag will contain messages for a single topic only.
  // First bag will contain high frequency topic, second and third bags - low frequency topics.
  // Start times are staggered to ensure that topics are interleaved during playback.
  // std::vector<tuple<topic_name, topic_period_ms, num_msgs, start_time>>
  std::vector<tuple<std::string, rcutils_duration_value_t, size_t, int64_t>> bag_descriptors{
    {high_freq_topic1_name, high_freq_topic_period_ms, num_high_freq_msgs_per_bag, 1},
    {low_freq_topic1_name, low_freq_topic_period_ms, num_low_freq_msgs_per_bag, 5},
    {low_freq_topic2_name, low_freq_topic_period_ms, num_low_freq_msgs_per_bag, 10}
  };

  std::vector<rosbag2_transport::Player::reader_storage_options_pair_t> bags{};
  std::size_t total_messages = 0u;
  for (const auto & [topic_name, topic_period_ms, num_msgs, start_time] : bag_descriptors) {
    std::vector<rosbag2_storage::TopicMetadata> topics{
      {1u, topic_name, "test_msgs/msg/BasicTypes", "", {}, ""}
    };
    std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> msgs;
    msgs.reserve(num_msgs);
    msgs.emplace_back(serialize_test_message(topic_name, start_time, msg));
    for (size_t i = 1; i < num_msgs; i++) {
      auto last_msg = msgs.back();
      msgs.emplace_back(
        serialize_test_message(
          topic_name,
          RCUTILS_NS_TO_MS(last_msg->recv_timestamp) + topic_period_ms,
          msg));
    }
    total_messages += msgs.size();
    auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
    prepared_mock_reader->prepare(msgs, topics);
    bags.emplace_back(
      std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader)), storage_options_);
  }
  ASSERT_EQ(total_messages, num_high_freq_msgs_per_bag + 2 * num_low_freq_msgs_per_bag);

  // Keep read_ahead_queue_size small to trigger possible starvation
  play_options_.read_ahead_queue_size = num_low_freq_msgs_per_bag / 2;
  auto player = std::make_shared<rosbag2_transport::Player>(std::move(bags), play_options_);

  using timestamped_msg_t =
    std::pair<std::chrono::steady_clock::time_point,
      std::shared_ptr<rosbag2_storage::SerializedBagMessage>>;

  std::unordered_map<std::string, std::vector<timestamped_msg_t>> published_msgs_per_topic;
  std::size_t num_played_messages = 0u;
  const auto callback = [&](std::shared_ptr<rosbag2_storage::SerializedBagMessage> msg)
    {
      num_played_messages++;
      published_msgs_per_topic[msg->topic_name].emplace_back(std::chrono::steady_clock::now(), msg);
    };
  player->add_on_play_message_pre_callback(callback);
  player->play();
  ASSERT_TRUE(player->wait_for_playback_to_start(10s));
  ASSERT_TRUE(player->wait_for_playback_to_finish(10s));
  EXPECT_EQ(total_messages, num_played_messages);
  EXPECT_EQ(published_msgs_per_topic.size(), bag_descriptors.size());

  for (const auto & [topic_name, topic_period_ms, num_msgs, start_time] : bag_descriptors) {
    ASSERT_TRUE(published_msgs_per_topic.find(topic_name) != published_msgs_per_topic.end());
    EXPECT_EQ(published_msgs_per_topic[topic_name].size(), num_msgs)
            << "Unexpected number of messages for topic " << topic_name;
  }
  // Verify the time intervals between published messages per topic
  // Allow for a tolerance of high_freq_topic_period_ms / 2 = 10 ms
  const rcutils_duration_value_t tolerance_ms = high_freq_topic_period_ms / 2;  // 10 ms;
  for (const auto & [topic_name, published_msgs] : published_msgs_per_topic) {
    if (topic_name == high_freq_topic1_name) {
      // High frequency topic, expect messages to be published at intervals of about
      // high_freq_topic_period_ms
      for (size_t i = 1; i < published_msgs.size(); i++) {
        auto time_diff = std::chrono::duration_cast<std::chrono::milliseconds>(
          published_msgs[i].first - published_msgs[i - 1].first).count();
        EXPECT_THAT(time_diff,
                    AllOf(Ge(high_freq_topic_period_ms - tolerance_ms),
                          Le(high_freq_topic_period_ms + tolerance_ms))
        ) << "msg_number=" << i << ", time_diff=" << time_diff;
      }
    } else if (topic_name == low_freq_topic1_name || topic_name == low_freq_topic2_name) {
      // Low frequency topics, expect messages to be published at intervals of about
      // low_freq_topic_period_ms
      for (size_t i = 1; i < published_msgs.size(); i++) {
        auto time_diff = std::chrono::duration_cast<std::chrono::milliseconds>(
          published_msgs[i].first - published_msgs[i - 1].first).count();
        EXPECT_THAT(time_diff,
                    AllOf(Ge(low_freq_topic_period_ms - tolerance_ms),
                          Le(low_freq_topic_period_ms + tolerance_ms))
        ) << "msg_number=" << i << ", time_diff=" << time_diff;
      }
    } else {
      FAIL() << "Unexpected topic name: " << topic_name;
    }
  }
}

TEST_F(RosBag2PlayTestFixture, recorded_messages_are_played_for_all_services)
{
  const std::string service_name1 = "/test_service1";
  const std::string service_event_name1 = service_name1 + "/_service_event";
  const std::string service_name2 = "/test_service2";
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

  auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
  prepared_mock_reader->prepare(messages, services_types);
  auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));

  std::vector<std::shared_ptr<test_msgs::srv::BasicTypes::Request>> service1_receive_requests;
  std::vector<std::shared_ptr<test_msgs::srv::BasicTypes::Request>> service2_receive_requests;

  srv_->setup_service<test_msgs::srv::BasicTypes>(service_name1, service1_receive_requests);
  srv_->setup_service<test_msgs::srv::BasicTypes>(service_name2, service2_receive_requests);

  srv_->run_services();

  ASSERT_TRUE(srv_->all_services_ready());

  play_options_.publish_service_requests = true;
  auto player =
    std::make_shared<rosbag2_transport::Player>(std::move(reader), storage_options_, play_options_);

  spin_thread_and_wait_for_sent_service_requests_to_finish(player, {service_name1, service_name2});

  EXPECT_EQ(service1_receive_requests.size(), 2);
  EXPECT_EQ(service2_receive_requests.size(), 2);
}

TEST_F(RosBag2PlayTestFixture, recorded_messages_are_played_for_topics_and_services)
{
  auto topic_msg = get_messages_basic_types()[0];
  topic_msg->int64_value = 1111;

  const std::string topic_name = "/topic1";
  const std::string service_name = "/test_service1";
  const std::string service_event_name = service_name + "/_service_event";

  auto services_types = std::vector<rosbag2_storage::TopicMetadata>{
    {1u, topic_name, "test_msgs/BasicTypes", "", {}, ""},
    {2u, service_event_name, "test_msgs/srv/BasicTypes_Event", "", {}, ""},
  };
  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages =
  {
    serialize_test_message(service_event_name, 500, get_service_event_message_basic_types()[0]),
    serialize_test_message(topic_name, 600, topic_msg),
    serialize_test_message(service_event_name, 550, get_service_event_message_basic_types()[1]),
    serialize_test_message(topic_name, 400, topic_msg),
  };

  auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
  prepared_mock_reader->prepare(messages, services_types);
  auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));

  std::vector<std::shared_ptr<test_msgs::srv::BasicTypes::Request>> service_receive_requests;
  srv_->setup_service<test_msgs::srv::BasicTypes>(service_name, service_receive_requests);
  srv_->run_services();
  ASSERT_TRUE(srv_->all_services_ready());

  sub_->add_subscription<test_msgs::msg::BasicTypes>(topic_name, 2);
  auto await_received_messages = sub_->spin_subscriptions();

  play_options_.publish_service_requests = true;
  auto player = std::make_shared<rosbag2_transport::Player>(
    std::move(reader), storage_options_, play_options_);

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(player);
  auto spin_thread = std::thread([&exec]() {exec.spin();});

  player->play();
  player->wait_for_playback_to_finish();

  await_received_messages.get();

  auto replayed_topic_msg = sub_->get_received_messages<test_msgs::msg::BasicTypes>(topic_name);
  EXPECT_THAT(replayed_topic_msg, SizeIs(Ge(2u)));
  EXPECT_THAT(
    replayed_topic_msg,
    Each(Pointee(Field(&test_msgs::msg::BasicTypes::int64_value, 1111))));

  EXPECT_TRUE(player->wait_for_sent_service_requests_to_finish(service_name, 2s));
  exec.cancel();
  spin_thread.join();
  ASSERT_EQ(service_receive_requests.size(), 2);
  for (size_t i = 0; i < service_receive_requests.size(); i++) {
    EXPECT_EQ(
      service_receive_requests[i]->int32_value,
      get_service_event_message_basic_types()[i]->request[0].int32_value);
    EXPECT_EQ(
      service_receive_requests[i]->int64_value,
      get_service_event_message_basic_types()[i]->request[0].int64_value);
  }
}

TEST_F(RosBag2PlayTestFixture, recorded_messages_are_played_for_all_topics_with_unknown_type)
{
  auto primitive_message1 = get_messages_basic_types()[0];
  primitive_message1->int32_value = 42;

  auto complex_message1 = get_messages_arrays()[0];
  complex_message1->float32_values = {{40.0f, 2.0f, 0.0f}};
  complex_message1->bool_values = {{true, false, true}};

  auto unknown_message1 = get_messages_basic_types()[0];
  unknown_message1->int32_value = 42;

  auto topic_types = std::vector<rosbag2_storage::TopicMetadata>{
    {1u, "topic1", "test_msgs/BasicTypes", "", {}, ""},
    {2u, "topic2", "test_msgs/Arrays", "", {}, ""},
    {3u, "topic3", "unknown_msgs/UnknownType", "", {}, ""},
  };

  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages =
  {serialize_test_message("topic1", 500, primitive_message1),
    serialize_test_message("topic1", 700, primitive_message1),
    serialize_test_message("topic1", 900, primitive_message1),
    serialize_test_message("topic2", 550, complex_message1),
    serialize_test_message("topic2", 750, complex_message1),
    serialize_test_message("topic2", 950, complex_message1),
    serialize_test_message("topic3", 900, unknown_message1)};

  auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
  prepared_mock_reader->prepare(messages, topic_types);
  auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));

  // Due to a problem related to the subscriber, we play many (3) messages but make the subscriber
  // node spin only until 2 have arrived. Hence the 2 as `launch_subscriber()` argument.
  sub_->add_subscription<test_msgs::msg::BasicTypes>("/topic1", 2);
  sub_->add_subscription<test_msgs::msg::Arrays>("/topic2", 2);

  auto await_received_messages = sub_->spin_subscriptions();

  auto player = std::make_shared<rosbag2_transport::Player>(
    std::move(reader), storage_options_, play_options_);
  player->play();
  player->wait_for_playback_to_finish();
  await_received_messages.get();

  auto replayed_test_primitives = sub_->get_received_messages<test_msgs::msg::BasicTypes>(
    "/topic1");
  EXPECT_THAT(replayed_test_primitives, SizeIs(Ge(2u)));
  EXPECT_THAT(
    replayed_test_primitives,
    Each(Pointee(Field(&test_msgs::msg::BasicTypes::int32_value, 42))));

  auto replayed_test_arrays = sub_->get_received_messages<test_msgs::msg::Arrays>(
    "/topic2");
  EXPECT_THAT(replayed_test_arrays, SizeIs(Ge(2u)));
  EXPECT_THAT(
    replayed_test_arrays,
    Each(
      Pointee(
        Field(
          &test_msgs::msg::Arrays::bool_values,
          ElementsAre(true, false, true)))));
  EXPECT_THAT(
    replayed_test_arrays,
    Each(
      Pointee(
        Field(
          &test_msgs::msg::Arrays::float32_values,
          ElementsAre(40.0f, 2.0f, 0.0f)))));
}

TEST_F(RosBag2PlayTestFixture, recorded_messages_are_played_for_filtered_topics)
{
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
  {serialize_test_message("/topic1", 500, primitive_message1),
    serialize_test_message("/topic1", 700, primitive_message1),
    serialize_test_message("/topic1", 900, primitive_message1),
    serialize_test_message("/topic2", 550, complex_message1),
    serialize_test_message("/topic2", 750, complex_message1),
    serialize_test_message("/topic2", 950, complex_message1)};

  // Filter allows /topic2, blocks /topic1
  {
    play_options_.topics_to_filter = {"topic2"};

    // SubscriptionManager has to be recreated for every unique test
    // If it isn't, message counts accumulate
    sub_.reset();
    sub_ = std::make_shared<SubscriptionManager>();
    sub_->add_subscription<test_msgs::msg::BasicTypes>("/topic1", 0);
    sub_->add_subscription<test_msgs::msg::Arrays>("/topic2", 2);

    auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
    prepared_mock_reader->prepare(messages, topic_types);
    auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));

    auto await_received_messages = sub_->spin_subscriptions();

    auto player = std::make_shared<rosbag2_transport::Player>(
      std::move(reader), storage_options_, play_options_);
    player->play();
    player->wait_for_playback_to_finish();
    await_received_messages.get();

    auto replayed_topic1 = sub_->get_received_messages<test_msgs::msg::BasicTypes>("/topic1");
    // No messages are allowed to have arrived
    EXPECT_THAT(replayed_topic1, SizeIs(0u));

    auto replayed_topic2 = sub_->get_received_messages<test_msgs::msg::Arrays>("/topic2");
    // All we care is that any messages arrived
    EXPECT_THAT(replayed_topic2, SizeIs(Ge(1u)));
  }

  // Filter allows /topic1, blocks /topic2
  {
    play_options_.topics_to_filter = {"topic1"};

    // SubscriptionManager has to be recreated for every unique test
    // otherwise counts accumulate, returning the spin immediately
    sub_.reset();
    sub_ = std::make_shared<SubscriptionManager>();
    sub_->add_subscription<test_msgs::msg::BasicTypes>("/topic1", 2);
    sub_->add_subscription<test_msgs::msg::Arrays>("/topic2", 0);

    auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
    prepared_mock_reader->prepare(messages, topic_types);
    auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));

    auto await_received_messages = sub_->spin_subscriptions();

    auto player = std::make_shared<rosbag2_transport::Player>(
      std::move(reader), storage_options_, play_options_);
    player->play();
    player->wait_for_playback_to_finish();
    await_received_messages.get();

    auto replayed_topic1 = sub_->get_received_messages<test_msgs::msg::BasicTypes>("/topic1");
    // All we care is that any messages arrived
    EXPECT_THAT(replayed_topic1, SizeIs(Ge(1u)));

    auto replayed_topic2 = sub_->get_received_messages<test_msgs::msg::Arrays>("/topic2");
    // No messages are allowed to have arrived
    EXPECT_THAT(replayed_topic2, SizeIs(0u));
  }

  // No filter, receive both topics
  {
    play_options_.topics_to_filter.clear();

    // SubscriptionManager has to be recreated for every unique test
    // otherwise counts accumulate, returning the spin immediately
    sub_.reset();
    sub_ = std::make_shared<SubscriptionManager>();
    sub_->add_subscription<test_msgs::msg::BasicTypes>("/topic1", 2);
    sub_->add_subscription<test_msgs::msg::Arrays>("/topic2", 2);

    auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
    prepared_mock_reader->prepare(messages, topic_types);
    auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));

    auto await_received_messages = sub_->spin_subscriptions();

    auto player = std::make_shared<rosbag2_transport::Player>(
      std::move(reader), storage_options_, play_options_);
    player->play();
    player->wait_for_playback_to_finish();
    await_received_messages.get();

    auto replayed_topic1 = sub_->get_received_messages<test_msgs::msg::BasicTypes>("/topic1");
    // All we care is that any messages arrived
    EXPECT_THAT(replayed_topic1, SizeIs(Ge(1u)));

    auto replayed_topic2 = sub_->get_received_messages<test_msgs::msg::Arrays>("/topic2");
    // All we care is that any messages arrived
    EXPECT_THAT(replayed_topic2, SizeIs(Ge(1u)));
  }
}

TEST_F(RosBag2PlayTestFixture, recorded_messages_are_played_for_filtered_services)
{
  const std::string service_name1 = "/test_service1";
  const std::string service_event_name1 = service_name1 + "/_service_event";
  const std::string service_name2 = "/test_service2";
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

  play_options_.publish_service_requests = true;

  // Filter allows /test_service2, blocks /test_service1
  {
    play_options_.services_to_filter = {service_event_name2};

    srv_.reset();
    srv_ = std::make_shared<ServiceManager>();

    std::vector<std::shared_ptr<test_msgs::srv::BasicTypes::Request>> service1_receive_requests;
    std::vector<std::shared_ptr<test_msgs::srv::BasicTypes::Request>> service2_receive_requests;
    srv_->setup_service<test_msgs::srv::BasicTypes>(service_name1, service1_receive_requests);
    srv_->setup_service<test_msgs::srv::BasicTypes>(service_name2, service2_receive_requests);

    srv_->run_services();
    ASSERT_TRUE(srv_->all_services_ready());

    auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
    prepared_mock_reader->prepare(messages, services_types);
    auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));

    auto player = std::make_shared<rosbag2_transport::Player>(
      std::move(reader), storage_options_, play_options_);

    // Only need to wait for sent service 2 request to finish
    spin_thread_and_wait_for_sent_service_requests_to_finish(player, {service_name2});

    EXPECT_EQ(service1_receive_requests.size(), 0);
    EXPECT_EQ(service2_receive_requests.size(), 2);
  }

  // Filter allows /test_service1, blocks /test_service2
  {
    play_options_.services_to_filter = {service_event_name1};

    srv_.reset();
    srv_ = std::make_shared<ServiceManager>();
    std::vector<std::shared_ptr<test_msgs::srv::BasicTypes::Request>> service1_receive_requests;
    std::vector<std::shared_ptr<test_msgs::srv::BasicTypes::Request>> service2_receive_requests;
    srv_->setup_service<test_msgs::srv::BasicTypes>(service_name1, service1_receive_requests);
    srv_->setup_service<test_msgs::srv::BasicTypes>(service_name2, service2_receive_requests);

    srv_->run_services();
    ASSERT_TRUE(srv_->all_services_ready());

    auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
    prepared_mock_reader->prepare(messages, services_types);
    auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));

    auto player = std::make_shared<rosbag2_transport::Player>(
      std::move(reader), storage_options_, play_options_);
    // Only need to wait for sent service 1 request to finish
    spin_thread_and_wait_for_sent_service_requests_to_finish(player, {service_name1});

    EXPECT_EQ(service1_receive_requests.size(), 2);
    EXPECT_EQ(service2_receive_requests.size(), 0);
  }

  // No filter, receive both services
  {
    play_options_.services_to_filter.clear();

    srv_.reset();
    srv_ = std::make_shared<ServiceManager>();
    std::vector<std::shared_ptr<test_msgs::srv::BasicTypes::Request>> service1_receive_requests;
    std::vector<std::shared_ptr<test_msgs::srv::BasicTypes::Request>> service2_receive_requests;
    srv_->setup_service<test_msgs::srv::BasicTypes>(service_name1, service1_receive_requests);
    srv_->setup_service<test_msgs::srv::BasicTypes>(service_name2, service2_receive_requests);

    srv_->run_services();
    ASSERT_TRUE(srv_->all_services_ready());

    auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
    prepared_mock_reader->prepare(messages, services_types);
    auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));

    auto player = std::make_shared<rosbag2_transport::Player>(
      std::move(reader), storage_options_, play_options_);
    spin_thread_and_wait_for_sent_service_requests_to_finish(
      player, {service_name1, service_name2});

    EXPECT_EQ(service1_receive_requests.size(), 2);
    EXPECT_EQ(service2_receive_requests.size(), 2);
  }
}

TEST_F(RosBag2PlayTestFixture, recorded_messages_are_played_for_filtered_topics_and_services)
{
  const std::string topic_name1 = "/topic1";
  const std::string topic_name2 = "/topic2";
  const std::string service_name1 = "/test_service1";
  const std::string service_event_name1 = service_name1 + "/_service_event";
  const std::string service_name2 = "/test_service2";
  const std::string service_event_name2 = service_name2 + "/_service_event";

  auto all_types = std::vector<rosbag2_storage::TopicMetadata>{
    {1u, topic_name1, "test_msgs/BasicTypes", "", {}, ""},
    {2u, topic_name2, "test_msgs/BasicTypes", "", {}, ""},
    {3u, service_event_name1, "test_msgs/srv/BasicTypes_Event", "", {}, ""},
    {4u, service_event_name2, "test_msgs/srv/BasicTypes_Event", "", {}, ""},
  };

  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages =
  {
    serialize_test_message(topic_name1, 500, get_messages_basic_types()[0]),
    serialize_test_message(service_event_name1, 520, get_service_event_message_basic_types()[0]),
    serialize_test_message(topic_name2, 520, get_messages_basic_types()[0]),
    serialize_test_message(service_event_name2, 550, get_service_event_message_basic_types()[0]),
  };

  play_options_.publish_service_requests = true;
  // Filter allows all topics, blocks service test_service2
  {
    play_options_.topics_to_filter = {topic_name1, topic_name2};
    play_options_.services_to_filter = {service_event_name1};

    sub_.reset();
    sub_ = std::make_shared<SubscriptionManager>();
    sub_->add_subscription<test_msgs::msg::BasicTypes>(topic_name1, 1);
    sub_->add_subscription<test_msgs::msg::BasicTypes>(topic_name2, 1);

    srv_.reset();
    srv_ = std::make_shared<ServiceManager>();
    std::vector<std::shared_ptr<test_msgs::srv::BasicTypes::Request>> service1_receive_requests;
    std::vector<std::shared_ptr<test_msgs::srv::BasicTypes::Request>> service2_receive_requests;
    srv_->setup_service<test_msgs::srv::BasicTypes>(service_name1, service1_receive_requests);
    srv_->setup_service<test_msgs::srv::BasicTypes>(service_name2, service2_receive_requests);
    srv_->run_services();
    ASSERT_TRUE(srv_->all_services_ready());

    auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
    prepared_mock_reader->prepare(messages, all_types);
    auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));

    auto await_received_messages = sub_->spin_subscriptions();

    auto player = std::make_shared<rosbag2_transport::Player>(
      std::move(reader), storage_options_, play_options_);

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(player);
    auto spin_thread = std::thread([&exec]() {exec.spin();});

    player->play();
    await_received_messages.get();
    player->wait_for_playback_to_finish();

    EXPECT_TRUE(player->wait_for_sent_service_requests_to_finish(service_name1, 2s));
    exec.cancel();
    spin_thread.join();

    // Filter allow all topics
    auto replayed_topic1 = sub_->get_received_messages<test_msgs::msg::BasicTypes>(topic_name1);
    EXPECT_THAT(replayed_topic1, SizeIs(1u));
    auto replayed_topic2 = sub_->get_received_messages<test_msgs::msg::BasicTypes>(topic_name2);
    EXPECT_THAT(replayed_topic2, SizeIs(1u));

    // Filter allow test_service1, block test_service2
    EXPECT_EQ(service1_receive_requests.size(), 1);
    EXPECT_EQ(service2_receive_requests.size(), 0);
  }

  // Filter allows all services, blocks topic2
  {
    play_options_.topics_to_filter = {topic_name1};
    play_options_.services_to_filter = {
      service_event_name1, service_event_name2};

    sub_.reset();
    sub_ = std::make_shared<SubscriptionManager>();
    sub_->add_subscription<test_msgs::msg::BasicTypes>(topic_name1, 1);
    sub_->add_subscription<test_msgs::msg::BasicTypes>(topic_name2, 0);

    srv_.reset();
    srv_ = std::make_shared<ServiceManager>();
    std::vector<std::shared_ptr<test_msgs::srv::BasicTypes::Request>> service1_receive_requests;
    std::vector<std::shared_ptr<test_msgs::srv::BasicTypes::Request>> service2_receive_requests;
    srv_->setup_service<test_msgs::srv::BasicTypes>(service_name1, service1_receive_requests);
    srv_->setup_service<test_msgs::srv::BasicTypes>(service_name2, service2_receive_requests);
    srv_->run_services();
    ASSERT_TRUE(srv_->all_services_ready());

    auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
    prepared_mock_reader->prepare(messages, all_types);
    auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));

    auto await_received_messages = sub_->spin_subscriptions();

    auto player = std::make_shared<rosbag2_transport::Player>(
      std::move(
        reader), storage_options_, play_options_);

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(player);
    auto spin_thread = std::thread([&exec]() {exec.spin();});

    player->play();
    await_received_messages.get();
    player->wait_for_playback_to_finish();

    EXPECT_TRUE(player->wait_for_sent_service_requests_to_finish(service_name1, 2s));
    EXPECT_TRUE(player->wait_for_sent_service_requests_to_finish(service_name2, 2s));
    exec.cancel();
    spin_thread.join();

    // Filter allow topic2, block topic1
    auto replayed_topic1 = sub_->get_received_messages<test_msgs::msg::BasicTypes>(topic_name1);
    EXPECT_THAT(replayed_topic1, SizeIs(1u));
    auto replayed_topic2 = sub_->get_received_messages<test_msgs::msg::BasicTypes>(topic_name2);
    EXPECT_THAT(replayed_topic2, SizeIs(0u));

    // Filter allow all services
    EXPECT_EQ(service1_receive_requests.size(), 1);
    EXPECT_EQ(service2_receive_requests.size(), 1);
  }

  // Filter allows all services and topics
  {
    play_options_.topics_to_filter = {topic_name1, topic_name2};
    play_options_.services_to_filter = {
      service_event_name1, service_event_name2};

    sub_.reset();
    sub_ = std::make_shared<SubscriptionManager>();
    sub_->add_subscription<test_msgs::msg::BasicTypes>(topic_name1, 1);
    sub_->add_subscription<test_msgs::msg::BasicTypes>(topic_name2, 1);

    srv_.reset();
    srv_ = std::make_shared<ServiceManager>();
    std::vector<std::shared_ptr<test_msgs::srv::BasicTypes::Request>> service1_receive_requests;
    std::vector<std::shared_ptr<test_msgs::srv::BasicTypes::Request>> service2_receive_requests;
    srv_->setup_service<test_msgs::srv::BasicTypes>(service_name1, service1_receive_requests);
    srv_->setup_service<test_msgs::srv::BasicTypes>(service_name2, service2_receive_requests);
    srv_->run_services();
    ASSERT_TRUE(srv_->all_services_ready());

    auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
    prepared_mock_reader->prepare(messages, all_types);
    auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));

    auto await_received_messages = sub_->spin_subscriptions();

    auto player = std::make_shared<rosbag2_transport::Player>(
      std::move(reader), storage_options_, play_options_);

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(player);
    auto spin_thread = std::thread([&exec]() {exec.spin();});

    player->play();
    await_received_messages.get();
    player->wait_for_playback_to_finish();

    EXPECT_TRUE(player->wait_for_sent_service_requests_to_finish(service_name1, 2s));
    EXPECT_TRUE(player->wait_for_sent_service_requests_to_finish(service_name2, 2s));
    exec.cancel();
    spin_thread.join();

    // Filter allow all topics
    auto replayed_topic1 = sub_->get_received_messages<test_msgs::msg::BasicTypes>("/topic1");
    EXPECT_THAT(replayed_topic1, SizeIs(1u));
    auto replayed_topic2 = sub_->get_received_messages<test_msgs::msg::BasicTypes>("/topic2");
    EXPECT_THAT(replayed_topic2, SizeIs(1u));

    // Filter allow all services
    EXPECT_EQ(service1_receive_requests.size(), 1);
    EXPECT_EQ(service2_receive_requests.size(), 1);
  }
}

TEST_F(RosBag2PlayTestFixture, recorded_messages_are_played_for_filtered_topics_with_unknown_type)
{
  auto primitive_message1 = get_messages_basic_types()[0];
  primitive_message1->int32_value = 42;

  auto complex_message1 = get_messages_arrays()[0];
  complex_message1->float32_values = {{40.0f, 2.0f, 0.0f}};
  complex_message1->bool_values = {{true, false, true}};

  auto unknown_message1 = get_messages_basic_types()[0];
  unknown_message1->int32_value = 42;

  auto topic_types = std::vector<rosbag2_storage::TopicMetadata>{
    {1u, "/topic1", "test_msgs/BasicTypes", "", {}, ""},
    {2u, "/topic2", "test_msgs/Arrays", "", {}, ""},
    {3u, "/topic3", "unknown_msgs/UnknownType", "", {}, ""},
  };

  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages =
  {serialize_test_message("/topic1", 500, primitive_message1),
    serialize_test_message("/topic1", 700, primitive_message1),
    serialize_test_message("/topic1", 900, primitive_message1),
    serialize_test_message("/topic2", 550, complex_message1),
    serialize_test_message("/topic2", 750, complex_message1),
    serialize_test_message("/topic2", 950, complex_message1),
    serialize_test_message("/topic3", 900, unknown_message1)};

  {
    play_options_.topics_to_filter = {"topic2"};

    auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
    prepared_mock_reader->prepare(messages, topic_types);
    auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));

    // Due to a problem related to the subscriber, we play many (3) messages but make the subscriber
    // node spin only until 2 have arrived. Hence the 2 as `launch_subscriber()` argument.

    sub_->add_subscription<test_msgs::msg::Arrays>("/topic2", 2);
    auto await_received_messages = sub_->spin_subscriptions();

    auto player = std::make_shared<rosbag2_transport::Player>(
      std::move(reader), storage_options_, play_options_);
    player->play();
    player->wait_for_playback_to_finish();
    await_received_messages.get();

    auto replayed_test_primitives = sub_->get_received_messages<test_msgs::msg::BasicTypes>(
      "/topic1");
    EXPECT_THAT(replayed_test_primitives, SizeIs(Ge(0u)));

    auto replayed_test_arrays = sub_->get_received_messages<test_msgs::msg::Arrays>(
      "/topic2");
    EXPECT_THAT(replayed_test_arrays, SizeIs(Ge(2u)));
  }

  // Set new filter
  {
    play_options_.topics_to_filter = {"topic1"};

    auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
    prepared_mock_reader->prepare(messages, topic_types);
    auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));

    sub_.reset();
    sub_ = std::make_shared<SubscriptionManager>();
    sub_->add_subscription<test_msgs::msg::BasicTypes>("/topic1", 2);

    auto await_received_messages = sub_->spin_subscriptions();

    auto player = std::make_shared<rosbag2_transport::Player>(
      std::move(reader), storage_options_, play_options_);
    player->play();
    player->wait_for_playback_to_finish();
    await_received_messages.get();

    auto replayed_test_primitives = sub_->get_received_messages<test_msgs::msg::BasicTypes>(
      "/topic1");
    EXPECT_THAT(replayed_test_primitives, SizeIs(Ge(2u)));

    auto replayed_test_arrays = sub_->get_received_messages<test_msgs::msg::Arrays>(
      "/topic2");
    EXPECT_THAT(replayed_test_arrays, SizeIs(Ge(0u)));
  }

  // Reset filter
  {
    play_options_.topics_to_filter = {"topic1", "topic2"};

    auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
    prepared_mock_reader->prepare(messages, topic_types);
    auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));

    sub_.reset();
    sub_ = std::make_shared<SubscriptionManager>();
    sub_->add_subscription<test_msgs::msg::BasicTypes>("/topic1", 2);
    sub_->add_subscription<test_msgs::msg::Arrays>("/topic2", 2);

    auto await_received_messages = sub_->spin_subscriptions();

    auto player = std::make_shared<rosbag2_transport::Player>(
      std::move(reader), storage_options_, play_options_);
    player->play();
    player->wait_for_playback_to_finish();
    await_received_messages.get();

    auto replayed_test_primitives = sub_->get_received_messages<test_msgs::msg::BasicTypes>(
      "/topic1");
    EXPECT_THAT(replayed_test_primitives, SizeIs(Ge(2u)));

    auto replayed_test_arrays = sub_->get_received_messages<test_msgs::msg::Arrays>(
      "/topic2");
    EXPECT_THAT(replayed_test_arrays, SizeIs(Ge(2u)));
  }
}

TEST_F(RosBag2PlayTestFixture, player_gracefully_exit_by_rclcpp_shutdown_in_pause) {
  auto primitive_message1 = get_messages_basic_types()[0];
  primitive_message1->int32_value = 42;
  auto topic_types = std::vector<rosbag2_storage::TopicMetadata>{
    {1u, "topic1", "test_msgs/BasicTypes", "", {}, ""},
  };

  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages =
  {
    serialize_test_message("topic1", 500, primitive_message1),
    serialize_test_message("topic1", 700, primitive_message1)
  };

  auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
  prepared_mock_reader->prepare(messages, topic_types);
  auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));
  auto player = std::make_shared<MockPlayer>(std::move(reader), storage_options_, play_options_);

  player->pause();
  player->play();
  player->wait_for_playback_to_start();
  ASSERT_TRUE(player->is_paused());

  rclcpp::shutdown();
  player->wait_for_playback_to_finish();
}

TEST_F(RosBag2PlayTestFixture, play_service_requests_from_service_introspection_messages)
{
  const std::string service_name = "/test_service1";
  const std::string service_event_name = service_name + "/_service_event";
  auto services_types = std::vector<rosbag2_storage::TopicMetadata>{
    {1u, service_event_name, "test_msgs/srv/BasicTypes_Event", "", {}, ""},
  };
  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages =
  {
    serialize_test_message(service_event_name, 5, get_service_event_message_basic_types()[2]),
    serialize_test_message(service_event_name, 10, get_service_event_message_basic_types()[0]),
    serialize_test_message(service_event_name, 20, get_service_event_message_basic_types()[2]),
    serialize_test_message(service_event_name, 25, get_service_event_message_basic_types()[0]),
    serialize_test_message(service_event_name, 30, get_service_event_message_basic_types()[2]),
  };

  auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
  prepared_mock_reader->prepare(messages, services_types);
  auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));

  std::vector<std::shared_ptr<test_msgs::srv::BasicTypes::Request>> received_service_requests;

  srv_->setup_service<test_msgs::srv::BasicTypes>(service_name, received_service_requests);
  srv_->run_services();
  ASSERT_TRUE(srv_->all_services_ready());

  play_options_.publish_service_requests = true;
  play_options_.service_requests_source = ServiceRequestsSource::SERVICE_INTROSPECTION;

  auto player =
    std::make_shared<rosbag2_transport::Player>(std::move(reader), storage_options_, play_options_);

  spin_thread_and_wait_for_sent_service_requests_to_finish(player, {service_name});

  EXPECT_EQ(received_service_requests.size(), 2);
  // expected_request is ServiceEventInfo::REQUEST_RECEIVED
  const auto expected_request = get_service_event_message_basic_types()[0]->request[0];
  for (const auto & service_request : received_service_requests) {
    EXPECT_EQ(service_request->int32_value, expected_request.int32_value) <<
      service_request->string_value;
    EXPECT_EQ(service_request->int64_value, expected_request.int64_value) <<
      service_request->string_value;
  }
}

TEST_F(RosBag2PlayTestFixture, play_service_requests_from_client_introspection_messages)
{
  const std::string service_name = "/test_service1";
  const std::string service_event_name = service_name + "/_service_event";
  auto services_types = std::vector<rosbag2_storage::TopicMetadata>{
    {1u, service_event_name, "test_msgs/srv/BasicTypes_Event", "", {}, ""},
  };
  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages =
  {
    serialize_test_message(service_event_name, 5, get_service_event_message_basic_types()[0]),
    serialize_test_message(service_event_name, 10, get_service_event_message_basic_types()[1]),
    serialize_test_message(service_event_name, 20, get_service_event_message_basic_types()[2]),
    serialize_test_message(service_event_name, 25, get_service_event_message_basic_types()[2]),
    serialize_test_message(service_event_name, 30, get_service_event_message_basic_types()[1]),
  };

  auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
  prepared_mock_reader->prepare(messages, services_types);
  auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));

  std::vector<std::shared_ptr<test_msgs::srv::BasicTypes::Request>> received_service_requests;

  srv_->setup_service<test_msgs::srv::BasicTypes>(service_name, received_service_requests);
  srv_->run_services();
  ASSERT_TRUE(srv_->all_services_ready());

  play_options_.publish_service_requests = true;
  play_options_.service_requests_source = ServiceRequestsSource::CLIENT_INTROSPECTION;

  auto player =
    std::make_shared<rosbag2_transport::Player>(std::move(reader), storage_options_, play_options_);

  spin_thread_and_wait_for_sent_service_requests_to_finish(player, {service_name});

  EXPECT_EQ(received_service_requests.size(), 2);
  // expected_request is ServiceEventInfo::REQUEST_SENT
  const auto expected_request = get_service_event_message_basic_types()[2]->request[0];
  for (const auto & service_request : received_service_requests) {
    EXPECT_EQ(service_request->int32_value, expected_request.int32_value) <<
      service_request->string_value;
    EXPECT_EQ(service_request->int64_value, expected_request.int64_value) <<
      service_request->string_value;
  }
}

TEST_F(RosBag2PlayTestFixture, play_service_events_and_topics)
{
  const std::string topic_1_name = "/topic1";
  const std::string topic_2_name = "/topic2";
  const std::string service_event_1_name = "/test_service1/_service_event";
  const std::string service_event_2_name = "/test_service2/_service_event";

  auto all_types = std::vector<rosbag2_storage::TopicMetadata>{
    {1u, topic_1_name, "test_msgs/BasicTypes", "", {}, ""},
    {2u, topic_2_name, "test_msgs/BasicTypes", "", {}, ""},
    {3u, service_event_1_name, "test_msgs/srv/BasicTypes_Event", "", {}, ""},
    {4u, service_event_2_name, "test_msgs/srv/BasicTypes_Event", "", {}, ""},
  };

  auto request_received_service_event = get_service_event_message_basic_types()[0];
  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages =
  {
    serialize_test_message(topic_1_name, 10, get_messages_basic_types()[0]),
    serialize_test_message(service_event_1_name, 20, request_received_service_event),
    serialize_test_message(topic_2_name, 30, get_messages_basic_types()[0]),
    serialize_test_message(service_event_2_name, 40, request_received_service_event),
  };

  play_options_.publish_service_requests = false;

  sub_ = std::make_shared<SubscriptionManager>();
  sub_->add_subscription<test_msgs::msg::BasicTypes>(topic_1_name, 1);
  sub_->add_subscription<test_msgs::msg::BasicTypes>(topic_2_name, 1);
  sub_->add_subscription<test_msgs::srv::BasicTypes_Event>(service_event_1_name, 1);
  sub_->add_subscription<test_msgs::srv::BasicTypes_Event>(service_event_2_name, 1);

  auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
  prepared_mock_reader->prepare(messages, all_types);
  auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));
  auto player = std::make_shared<MockPlayer>(std::move(reader), storage_options_, play_options_);

  // Wait for discovery to match publishers with subscribers
  ASSERT_TRUE(
    sub_->spin_and_wait_for_matched(player->get_list_of_publishers(), std::chrono::seconds(30)));

  auto await_received_messages = sub_->spin_subscriptions();

  player->play();
  player->wait_for_playback_to_finish();

  await_received_messages.get();

  auto replayed_topic1 = sub_->get_received_messages<test_msgs::msg::BasicTypes>(topic_1_name);
  EXPECT_THAT(replayed_topic1, SizeIs(1u));
  auto replayed_topic2 = sub_->get_received_messages<test_msgs::msg::BasicTypes>(topic_2_name);
  EXPECT_THAT(replayed_topic2, SizeIs(1u));
  auto replayed_service_event_1 =
    sub_->get_received_messages<test_msgs::srv::BasicTypes_Event>(service_event_1_name);
  EXPECT_THAT(replayed_service_event_1, SizeIs(1u));
  auto replayed_service_event_2 =
    sub_->get_received_messages<test_msgs::srv::BasicTypes_Event>(service_event_2_name);
  EXPECT_THAT(replayed_service_event_2, SizeIs(1u));
}

class RosBag2PlayQosOverrideTestFixture : public RosBag2PlayTestFixture
{
public:
  using Rosbag2QoS = rosbag2_storage::Rosbag2QoS;

  RosBag2PlayQosOverrideTestFixture()
  : RosBag2PlayTestFixture()
  {
  }

  void initialize(const std::vector<rclcpp::QoS> & offered_qos)
  {
    // Because these tests only cares about compatibility (receiving any messages at all)
    // We publish one more message than we expect to receive, to avoid caring about
    // shutdown edge behaviors that are not explicitly being tested here.
    const size_t num_msgs_to_publish = num_msgs_to_wait_for_ + 1;
    messages_.reserve(num_msgs_to_publish);
    for (size_t i = 0; i < num_msgs_to_publish; i++) {
      const auto timestamp = start_time_ms_ + message_spacing_ms_ * i;
      messages_.push_back(serialize_test_message(topic_name_, timestamp, basic_msg_));
    }

    topic_types_.push_back({0u, topic_name_, msg_type_, "", offered_qos, ""});
  }

  template<typename Duration>
  void play_and_wait(Duration timeout, bool expect_timeout = false)
  {
    auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
    prepared_mock_reader->prepare(messages_, topic_types_);
    auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));

    auto await_received_messages = sub_->spin_subscriptions();
    auto player = std::make_shared<rosbag2_transport::Player>(
      std::move(reader), storage_options_, play_options_);
    player->play();
    player->wait_for_playback_to_finish();
    const auto result = await_received_messages.wait_for(timeout);
    // Must EXPECT, can't ASSERT because transport needs to be shutdown if timed out
    if (expect_timeout) {
      EXPECT_EQ(result, std::future_status::timeout);
    } else {
      EXPECT_NE(result, std::future_status::timeout);
    }
    // Have to rclcpp::shutdown here to make the spin_subscriptions async thread exit
    rclcpp::shutdown();
  }

  const std::string topic_name_{"/test_topic"};
  const std::string msg_type_{"test_msgs/BasicTypes"};
  // Receiving _any_ messages means we've confirmed compatibility in these tests
  const size_t num_msgs_to_wait_for_{1};
  test_msgs::msg::BasicTypes::SharedPtr basic_msg_{get_messages_basic_types()[0]};
  std::vector<rosbag2_storage::TopicMetadata> topic_types_{};
  const int64_t start_time_ms_{500};
  const int64_t message_spacing_ms_{200};
  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages_;
};

TEST_F(RosBag2PlayQosOverrideTestFixture, topic_qos_profiles_overridden)
{
  // By default playback uses DURABILITY_VOLATILE.
  // When we request DURABILITY_TRANSIENT_LOCAL, we should expect no connection because that
  // request is incompatible.
  // However, if we override playback to offer DURABILITY_TRANSIENT_LOCAL, now we should expect
  // to receive messages.
  const auto qos_request = rclcpp::QoS{rclcpp::KeepAll()}.reliable().transient_local();
  const auto qos_playback_override = rclcpp::QoS{rclcpp::KeepAll()}.reliable().transient_local();
  const auto topic_qos_profile_overrides = std::unordered_map<std::string, rclcpp::QoS>{
    std::pair<std::string, rclcpp::QoS>{topic_name_, qos_playback_override},
  };
  // This should normally take less than 1s - just making it shorter than 60s default
  const auto timeout = 5s;

  initialize({});

  sub_->add_subscription<test_msgs::msg::BasicTypes>(
    topic_name_, num_msgs_to_wait_for_, qos_request);
  play_options_.topic_qos_profile_overrides = topic_qos_profile_overrides;

  // Fails if times out
  play_and_wait(timeout);
}

TEST_F(RosBag2PlayQosOverrideTestFixture, topic_qos_profiles_overridden_incompatible)
{
  // By default playback offers RELIABILITY_RELIABLE
  // We request RELIABILITY_RELIABLE here, which should be compatible.
  // However, we override the playback to offer RELIABILITY_BEST_EFFORT,
  // which should not be a compatible offer and therefore we should receive no messages.
  const auto qos_request = rclcpp::QoS{rclcpp::KeepAll()}.reliable();
  const auto qos_playback_override = rclcpp::QoS{rclcpp::KeepAll()}.best_effort();
  const auto topic_qos_profile_overrides = std::unordered_map<std::string, rclcpp::QoS>{
    std::pair<std::string, rclcpp::QoS>{topic_name_, qos_playback_override},
  };
  // If any messages were going to come in, it should happen in under 1s even in slow scenarios
  const auto timeout = 3s;

  initialize({});

  sub_->add_subscription<test_msgs::msg::BasicTypes>(
    topic_name_, num_msgs_to_wait_for_, qos_request);
  play_options_.topic_qos_profile_overrides = topic_qos_profile_overrides;

  // A timeout is expected if QoS is incompatible.
  bool expect_timeout = (rclcpp::qos_check_compatible(
    qos_playback_override, qos_request).compatibility != rclcpp::QoSCompatibility::Ok);

  play_and_wait(timeout, expect_timeout);
}

TEST_F(RosBag2PlayQosOverrideTestFixture, playback_uses_recorded_transient_local_profile)
{
  // In this test, we subscribe requesting DURABILITY_TRANSIENT_LOCAL.
  // The bag metadata has this recorded for the original Publisher,
  // so playback's offer should be compatible (whereas the default offer would not be)
  const auto transient_local_profile = Rosbag2QoS{Rosbag2QoS{}.transient_local()};
  // This should normally take less than 1s - just making it shorter than 60s default
  const auto timeout = 5s;

  initialize({transient_local_profile});

  sub_->add_subscription<test_msgs::msg::BasicTypes>(
    topic_name_, num_msgs_to_wait_for_, transient_local_profile);

  // Fails if times out
  play_and_wait(timeout);
}

TEST_F(RosBag2PlayQosOverrideTestFixture, playback_uses_recorded_deadline)
{
  // By default, QoS profiles use "unspecified/infinite" offers for duration-based policies
  // The subscription in this test requests a finite Deadline, and by receiving messages
  // we know that the playback has used the recorded finite Deadline duration.

  // The publisher is offering 2Hz. The subscription requests 1Hz, which is less strict of a
  // requirement, so they are compatible.
  const rclcpp::Duration request_deadline{1s};
  const rclcpp::Duration offer_deadline{500ms};
  const auto request_profile = Rosbag2QoS{}.deadline(request_deadline);
  const auto offer_profile = Rosbag2QoS{Rosbag2QoS{}.deadline(offer_deadline)};
  const auto timeout = 5s;

  initialize({offer_profile});
  sub_->add_subscription<test_msgs::msg::BasicTypes>(
    topic_name_, num_msgs_to_wait_for_, request_profile);
  play_and_wait(timeout);
}

TEST_F(RosBag2PlayQosOverrideTestFixture, override_has_precedence_over_recorded)
{
  // In this test, we show that the playback prefers the user override to the recorded values.
  // The subscription requests a Liveliness lease_duration that is shorter than the original
  // recorded publisher offered, so no messages should be passed.
  // However, the override to the publisher offers a shorter duration than the request, so
  // it should now be compatible.
  const rclcpp::Duration liveliness_request{500ms};
  const rclcpp::Duration recorded_liveliness_offer{1000ms};
  const rclcpp::Duration override_liveliness_offer{250ms};
  ASSERT_LT(liveliness_request, recorded_liveliness_offer);
  ASSERT_LT(override_liveliness_offer, liveliness_request);
  const auto request_profile = Rosbag2QoS{}.liveliness_lease_duration(liveliness_request);
  const auto recorded_offer_profile = Rosbag2QoS{Rosbag2QoS{}.liveliness_lease_duration(
      recorded_liveliness_offer)};
  const auto override_offer_profile = Rosbag2QoS{Rosbag2QoS{}.liveliness_lease_duration(
      override_liveliness_offer)};
  const auto topic_qos_profile_overrides = std::unordered_map<std::string, rclcpp::QoS>{
    std::pair<std::string, rclcpp::QoS>{topic_name_, override_offer_profile},
  };
  // This should normally take less than 1s - just making it shorter than 60s default
  const auto timeout = 5s;

  initialize({recorded_offer_profile});

  sub_->add_subscription<test_msgs::msg::BasicTypes>(
    topic_name_, num_msgs_to_wait_for_, request_profile);
  play_options_.topic_qos_profile_overrides = topic_qos_profile_overrides;

  // Fails if times out
  play_and_wait(timeout);
}

TEST_F(RosBag2PlayTestFixture, read_split_callback_is_called)
{
  auto topic_types = std::vector<rosbag2_storage::TopicMetadata>{
    {1u, "topic1", "test_msgs/BasicTypes", "", {}, ""},
  };

  auto prepared_mock_reader = std::make_unique<MockSequentialReader>();

  const size_t num_msgs_in_bag = prepared_mock_reader->max_messages_per_file() + 1;
  const int64_t start_time_ms = 100;
  const int64_t message_spacing_ms = 50;

  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages;
  messages.reserve(num_msgs_in_bag);

  auto primitive_message = get_messages_basic_types()[0];
  for (size_t i = 0; i < num_msgs_in_bag; i++) {
    primitive_message->int32_value = static_cast<int32_t>(i + 1);
    const int64_t timestamp = start_time_ms + message_spacing_ms * static_cast<int64_t>(i);
    messages.push_back(serialize_test_message("topic1", timestamp, primitive_message));
  }

  prepared_mock_reader->prepare(messages, topic_types);
  auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));

  bool callback_called = false;
  std::string closed_file, opened_file;
  rosbag2_cpp::bag_events::ReaderEventCallbacks callbacks;
  callbacks.read_split_callback =
    [&callback_called, &closed_file, &opened_file](rosbag2_cpp::bag_events::BagSplitInfo & info) {
      closed_file = info.closed_file;
      opened_file = info.opened_file;
      callback_called = true;
    };
  // This tests adding to the underlying prepared_mock_reader via the Reader instance
  reader->add_event_callbacks(callbacks);

  auto player = std::make_shared<MockPlayer>(std::move(reader), storage_options_, play_options_);

  sub_ = std::make_shared<SubscriptionManager>();
  sub_->add_subscription<test_msgs::msg::BasicTypes>("/topic1", messages.size());

  // Wait for discovery to match publishers with subscribers
  ASSERT_TRUE(
    sub_->spin_and_wait_for_matched(player->get_list_of_publishers(), std::chrono::seconds(30)));

  auto await_received_messages = sub_->spin_subscriptions();

  player->play();
  player->wait_for_playback_to_finish();
  await_received_messages.get();

  auto replayed_test_primitives = sub_->get_received_messages<test_msgs::msg::BasicTypes>(
    "/topic1");
  EXPECT_THAT(replayed_test_primitives, SizeIs(messages.size()));

  // Confirm that the callback was called and the file names have been sent with the event
  ASSERT_TRUE(callback_called);
  EXPECT_EQ(closed_file, "BagFile0");
  EXPECT_EQ(opened_file, "BagFile1");
}
