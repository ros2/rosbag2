// Copyright 2026 Apex.AI, Inc.
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
#include <memory>
#include <string>

#include "rcutils/time.h"
#include "rosbag2_storage/ros_helper.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"
#include "rosbag2_cpp/cache/message_cache_circular_buffer.hpp"

using namespace testing;  // NOLINT
using namespace std::chrono_literals;

// Introduce a fixture to host helpers.
class MessageCacheCircularBufferTest : public Test
{
protected:
  // Helper to construct a test message with given timestamp and content.
  static std::shared_ptr<rosbag2_storage::SerializedBagMessage> make_test_msg(
    rcutils_time_point_value_t timestamp_ns,
    const std::string & content = "test")
  {
    auto message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
    message->topic_name = "test_topic";
    message->recv_timestamp = timestamp_ns;
    message->serialized_data =
      rosbag2_storage::make_serialized_message(content.c_str(), content.length());
    return message;
  }
};

TEST_F(MessageCacheCircularBufferTest, constructor_throws_if_both_limits_are_zero) {
  EXPECT_THROW(
    rosbag2_cpp::cache::MessageCacheCircularBuffer(0, 0),
    std::invalid_argument);
}

TEST_F(MessageCacheCircularBufferTest, time_only_buffer_drops_old_messages_by_duration) {
  constexpr uint32_t max_duration_sec = 2;  // 2 seconds
  constexpr size_t max_size = 0;  // No size limit

  rosbag2_cpp::cache::MessageCacheCircularBuffer buffer(max_size, max_duration_sec);

  const auto base = 10s;
  // Push 10 messages spaced 500ms over 5 seconds total
  for (int i = 0; i < 10; i++) {
    auto ts = base + (i * 500ms);
    auto msg = make_test_msg(std::chrono::duration_cast<std::chrono::nanoseconds>(ts).count());
    ASSERT_TRUE(buffer.push(msg));
  }

  const auto data = buffer.data();
  ASSERT_FALSE(data.empty());

  const auto oldest_time = data.front()->recv_timestamp;
  const auto newest_time = data.back()->recv_timestamp;
  const auto duration_ns = newest_time - oldest_time;
  EXPECT_LE(duration_ns, RCUTILS_S_TO_NS(max_duration_sec));
}

TEST_F(MessageCacheCircularBufferTest, size_only_buffer_drops_old_messages_by_size) {
  constexpr uint32_t max_duration_sec = 0;   // No time limit
  constexpr size_t max_size = 100;           // 100 bytes

  rosbag2_cpp::cache::MessageCacheCircularBuffer buffer(max_size, max_duration_sec);

  // Push messages until the buffer enforces the size bound
  for (int i = 0; i < 20; i++) {
    const auto ts_ns = RCUTILS_MS_TO_NS(i);  // i ms -> ns
    auto msg = make_test_msg(ts_ns, std::string(30, 'x'));
    ASSERT_TRUE(buffer.push(msg));
  }

  auto data = buffer.data();
  ASSERT_FALSE(data.empty());

  size_t total_size = 0;
  for (const auto & msg : data) {
    total_size += msg->serialized_data->buffer_length;
  }
  EXPECT_LE(total_size, max_size);
}

TEST_F(MessageCacheCircularBufferTest, time_and_size_buffer_respects_both_limits) {
  constexpr uint32_t max_duration_sec = 3;  // 3 seconds
  constexpr size_t max_size = 200;          // 200 bytes

  rosbag2_cpp::cache::MessageCacheCircularBuffer buffer(max_size, max_duration_sec);

  const auto base = 1s;

  // Add many messages with ~50 bytes spaced by 100ms
  for (int i = 0; i < 50; i++) {
    auto ts = base + (i * 100ms);
    auto msg = make_test_msg(std::chrono::duration_cast<std::chrono::nanoseconds>(ts).count(),
                             std::string(50, 'x'));
    ASSERT_TRUE(buffer.push(msg));
  }

  const auto data = buffer.data();
  ASSERT_FALSE(data.empty());

  // Check size bound
  size_t total_size = 0;
  for (const auto & msg : data) {
    total_size += msg->serialized_data->buffer_length;
  }
  EXPECT_LE(total_size, max_size);

  // Check time bound
  const auto oldest_time = data.front()->recv_timestamp;
  const auto newest_time = data.back()->recv_timestamp;
  const auto duration_ns = newest_time - oldest_time;
  EXPECT_LE(duration_ns, RCUTILS_S_TO_NS(max_duration_sec));
}

TEST_F(MessageCacheCircularBufferTest, rejects_message_exceeding_size_limit) {
  constexpr uint32_t max_duration_sec = 10;
  constexpr size_t max_size = 50;

  rosbag2_cpp::cache::MessageCacheCircularBuffer buffer(max_size, max_duration_sec);

  const auto large_msg = make_test_msg(RCUTILS_S_TO_NS(1), std::string(100, 'x'));
  EXPECT_FALSE(buffer.push(large_msg));  // single message larger than buffer should be rejected
  EXPECT_EQ(buffer.size(), 0u);
}

TEST_F(MessageCacheCircularBufferTest, clear_empties_buffer) {
  constexpr uint32_t max_duration_sec = 5;
  constexpr size_t max_size = 500;

  rosbag2_cpp::cache::MessageCacheCircularBuffer buffer(max_size, max_duration_sec);

  for (int i = 0; i < 10; i++) {
    auto ts_ns = RCUTILS_MS_TO_NS(i);
    auto msg = make_test_msg(ts_ns);
    ASSERT_TRUE(buffer.push(msg));
  }
  EXPECT_GT(buffer.size(), 0u);

  buffer.clear();

  EXPECT_EQ(buffer.size(), 0u);
  EXPECT_TRUE(buffer.data().empty());
}

TEST_F(MessageCacheCircularBufferTest, handles_out_of_order_timestamps_gracefully) {
  constexpr uint32_t max_duration_sec = 2;
  constexpr size_t max_size = 0;

  rosbag2_cpp::cache::MessageCacheCircularBuffer buffer(max_size, max_duration_sec);

  const auto base = 5s;

  const auto newer_ts = base + 3s;
  const auto newer =
    make_test_msg(std::chrono::duration_cast<std::chrono::nanoseconds>(newer_ts).count());
  const auto older =
    make_test_msg(std::chrono::duration_cast<std::chrono::nanoseconds>(base).count());

  EXPECT_TRUE(buffer.push(newer));
  EXPECT_TRUE(buffer.push(older));

  EXPECT_EQ(buffer.size(), 2u);
}
