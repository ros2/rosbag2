// Copyright 2021 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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
#include <cmath>
#include <list>
#include <memory>
#include <string>
#include <utility>

#include "rcutils/time.h"
#include "rosbag2_storage/ros_helper.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"

#include "rosbag2_cpp/cache/circular_message_cache.hpp"

using namespace testing;  // NOLINT
using namespace std::chrono_literals;

namespace
{
inline size_t abs_diff(size_t a, size_t b)
{
  return a < b ? b - a : a - b;
}

// Helper to construct a test message with given timestamp and content.
std::shared_ptr<rosbag2_storage::SerializedBagMessage> make_test_msg(
  const rcutils_time_point_value_t timestamp_ns = 0,
  const std::string & content = "")
{
  static uint32_t counter = 0;
  // Fallback to a default unique message if no content provided
  std::string msg_content = (content.empty()) ? "Hello" + std::to_string(counter++) : content;

  auto message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  message->topic_name = "test_topic";
  message->recv_timestamp = timestamp_ns;
  message->serialized_data =
    rosbag2_storage::make_serialized_message(msg_content.c_str(), msg_content.length());
  return message;
}

std::string deserialize_message(std::shared_ptr<rcutils_uint8_array_t> serialized_message)
{
  std::unique_ptr<uint8_t[]> copied(new uint8_t[serialized_message->buffer_length + 1]);
  std::copy(
    serialized_message->buffer,
    serialized_message->buffer + serialized_message->buffer_length,
    copied.get());
  copied.get()[serialized_message->buffer_length] = '\0';
  std::string message_content(reinterpret_cast<char *>(copied.get()));
  return message_content;
}
}  // namespace

class CircularMessageCacheTest : public Test
{
public:
  CircularMessageCacheTest() {}

  virtual ~CircularMessageCacheTest() = default;

  const size_t cache_size_ {1 * 500};  // 500B cache
};

TEST_F(CircularMessageCacheTest, circular_message_cache_overwrites_old) {
  const unsigned message_count = 100;

  auto circular_message_cache = std::make_shared<rosbag2_cpp::cache::CircularMessageCache>(
    cache_size_);

  for (unsigned i = 0; i < message_count; ++i) {
    auto msg = make_test_msg();
    circular_message_cache->push(msg);
  }
  // Swap cache
  circular_message_cache->notify_data_ready();
  circular_message_cache->swap_buffers();

  auto consumer_buffer = circular_message_cache->get_consumer_buffer();
  EXPECT_THAT(consumer_buffer->size(), Ne(0u));

  auto message_vector = consumer_buffer->data();
  std::string first_message = deserialize_message(message_vector.front()->serialized_data);
  circular_message_cache->release_consumer_buffer();

  // Old messages should be dropped
  EXPECT_THAT(first_message, StrNe("Hello0"));

  size_t message_data_size = 0;

  for (auto & msg : message_vector) {
    message_data_size += msg->serialized_data->buffer_length;
  }

  size_t cache_size_diff = abs_diff(cache_size_, message_data_size);
  size_t allowed_diff{10};

  // Actual stored data size should be roughly the desired cache size
  EXPECT_THAT(cache_size_diff, Lt(allowed_diff));
}

TEST_F(CircularMessageCacheTest, circular_message_cache_ensure_empty) {
  const unsigned message_count = 100;

  auto circular_message_cache = std::make_shared<rosbag2_cpp::cache::CircularMessageCache>(
    cache_size_);

  for (unsigned i = 0; i < message_count; ++i) {
    auto msg = make_test_msg();
    circular_message_cache->push(msg);
  }
  // Swap filled cache to secondary
  circular_message_cache->notify_data_ready();
  circular_message_cache->swap_buffers();
  EXPECT_THAT(circular_message_cache->get_consumer_buffer()->size(), Ne(0u));
  circular_message_cache->release_consumer_buffer();

  // Swap back to primary (expected to empty buffer)
  circular_message_cache->notify_data_ready();
  circular_message_cache->swap_buffers();

  // Swap back to secondary without adding messages
  circular_message_cache->notify_data_ready();
  circular_message_cache->swap_buffers();
  // Cache should have been emptied
  EXPECT_THAT(circular_message_cache->get_consumer_buffer()->size(), Eq(0u));
  circular_message_cache->release_consumer_buffer();
}

TEST_F(CircularMessageCacheTest, circular_message_cache_rejects_null_message) {
  auto circular_message_cache = std::make_shared<rosbag2_cpp::cache::CircularMessageCache>(500);

  bool result = true;
  ASSERT_NO_THROW(result = circular_message_cache->push(nullptr));
  EXPECT_FALSE(result);

  // Test message with null serialized data
  auto msg = make_test_msg();
  msg->serialized_data = nullptr;
  ASSERT_NO_THROW(result = circular_message_cache->push(msg));
  EXPECT_FALSE(result);
}

TEST_F(CircularMessageCacheTest, constructor_throws_if_both_limits_are_zero) {
  EXPECT_THROW(rosbag2_cpp::cache::CircularMessageCache(0, 0), std::invalid_argument);
}

TEST_F(CircularMessageCacheTest, time_only_buffer_drops_old_messages_by_duration) {
  // No size limit, keep ~1s window
  const uint32_t max_buffer_duration = 1;  // 1 second
  auto circular_message_cache = std::make_shared<rosbag2_cpp::cache::CircularMessageCache>(
    /*max_buffer_size=*/0, max_buffer_duration);

  // Push messages with synthetic timestamps spaced 100ms
  const auto base = 1s;
  const auto messages_interval = 100ms;
  constexpr size_t message_count = 30;  // 3 seconds total
  const size_t expected_count = 1s / messages_interval + 1;  // Expect 11 messages in 1s window
  for (size_t i = 0; i < message_count; i++) {
    auto ts = base + (i * messages_interval);
    auto msg = make_test_msg(std::chrono::duration_cast<std::chrono::nanoseconds>(ts).count());
    EXPECT_TRUE(circular_message_cache->push(msg));
  }

  circular_message_cache->notify_data_ready();
  circular_message_cache->swap_buffers();

  auto consumer_buffer = circular_message_cache->get_consumer_buffer();
  auto messages = consumer_buffer->data();
  circular_message_cache->release_consumer_buffer();

  ASSERT_EQ(messages.size(), expected_count);
  const auto oldest = messages.front()->recv_timestamp;
  const auto newest = messages.back()->recv_timestamp;
  const auto span_ns = newest - oldest;
  EXPECT_GT(span_ns, 0);
  EXPECT_LE(span_ns, RCUTILS_S_TO_NS(max_buffer_duration));

  // Verify we kept the newest messages (last 1-second window)
  const auto expected_newest = base + ((message_count - 1) * messages_interval);
  EXPECT_EQ(newest, std::chrono::duration_cast<std::chrono::nanoseconds>(expected_newest).count());
  // Oldest should be 1 second before newest
  const auto expected_oldest = expected_newest - 1s;
  EXPECT_EQ(oldest, std::chrono::duration_cast<std::chrono::nanoseconds>(expected_oldest).count());
}

TEST_F(CircularMessageCacheTest, size_only_buffer_drops_old_messages_by_size) {
  // Size limit ~200 bytes, no time limit
  // 200 bytes ~28 messages with content = std::string(7, 'x'); (7 bytes + overhead)
  const size_t max_buffer_size = 200;
  auto circular_message_cache = std::make_shared<rosbag2_cpp::cache::CircularMessageCache>(
    max_buffer_size, /*max_buffer_duration=*/0);

  // Push many messages, ensure buffer doesn't grow over the size limit
  constexpr size_t message_count = 60;
  const auto messages_interval = 100ms;
  for (size_t i = 0; i < message_count; i++) {
    auto ts_ms = i * messages_interval;
    auto msg = make_test_msg(std::chrono::duration_cast<std::chrono::nanoseconds>(ts_ms).count(),
                             std::string(7, 'x'));
    EXPECT_TRUE(circular_message_cache->push(msg));
  }

  circular_message_cache->notify_data_ready();
  circular_message_cache->swap_buffers();

  auto consumer_buffer = circular_message_cache->get_consumer_buffer();
  auto messages = consumer_buffer->data();
  circular_message_cache->release_consumer_buffer();

  ASSERT_GT(messages.size(), 0u);
  size_t total_size = 0;
  for (const auto & m : messages) {
    total_size += m->serialized_data->buffer_length;
  }
  EXPECT_LE(total_size, max_buffer_size);
  const auto oldest = messages.front()->recv_timestamp;
  const auto newest = messages.back()->recv_timestamp;
  const auto span_ns = newest - oldest;
  EXPECT_GT(span_ns, 0);
}

TEST_F(CircularMessageCacheTest, time_and_size_buffer_respects_both_limits) {
  // Size limit ~200 bytes, time limit 2 seconds
  // 200 bytes ~28 messages with content = std::string(7, 'x'); (7 bytes + overhead)
  constexpr size_t max_buffer_size = 200;
  constexpr uint32_t max_buffer_duration = 2;  // 2 seconds
  auto circular_message_cache = std::make_shared<rosbag2_cpp::cache::CircularMessageCache>(
    max_buffer_size, max_buffer_duration);

  const auto base = 10s;
  constexpr size_t message_count = 100;  // > estimated 28 messages
  const auto messages_interval = 50ms;
  for (size_t i = 0; i < message_count; i++) {
    auto ts = base + (i * messages_interval);
    auto msg = make_test_msg(std::chrono::duration_cast<std::chrono::nanoseconds>(ts).count(),
                             std::string(7, 'x'));
    EXPECT_TRUE(circular_message_cache->push(msg));
  }

  circular_message_cache->notify_data_ready();
  circular_message_cache->swap_buffers();

  auto consumer_buffer = circular_message_cache->get_consumer_buffer();
  auto messages = consumer_buffer->data();
  circular_message_cache->release_consumer_buffer();

  ASSERT_GT(messages.size(), 0u);

  // Check size
  size_t total_size = 0;
  for (const auto & m : messages) {
    total_size += m->serialized_data->buffer_length;
  }
  EXPECT_LE(total_size, max_buffer_size);

  // Check time window
  const auto oldest = messages.front()->recv_timestamp;
  const auto newest = messages.back()->recv_timestamp;
  const auto span_ns = newest - oldest;
  EXPECT_GT(span_ns, 0);
  EXPECT_LE(span_ns, RCUTILS_S_TO_NS(max_buffer_duration));
}

TEST_F(CircularMessageCacheTest, rejects_message_exceeding_size_limit) {
  constexpr uint32_t max_duration_sec = 10;
  constexpr size_t max_buffer_size = 50;

  auto circular_message_cache = std::make_shared<rosbag2_cpp::cache::CircularMessageCache>(
    max_buffer_size, max_duration_sec);

  const auto large_msg = make_test_msg(RCUTILS_S_TO_NS(1), std::string(100, 'x'));
  // Push single message larger than buffer should be rejected
  EXPECT_FALSE(circular_message_cache->push(large_msg));

  circular_message_cache->notify_data_ready();
  circular_message_cache->swap_buffers();

  auto buffer = circular_message_cache->get_consumer_buffer()->data();
  EXPECT_EQ(buffer.size(), 0u);
}

TEST_F(CircularMessageCacheTest, handles_out_of_order_timestamps_gracefully) {
  constexpr uint32_t max_duration_sec = 2;
  constexpr size_t max_buffer_size = 0;

  auto circular_message_cache = std::make_shared<rosbag2_cpp::cache::CircularMessageCache>(
    max_buffer_size, max_duration_sec);

  const auto base = 5s;
  const auto newer_ts = base + 3s;
  const auto newer_message =
    make_test_msg(std::chrono::duration_cast<std::chrono::nanoseconds>(newer_ts).count());
  const auto older_message =
    make_test_msg(std::chrono::duration_cast<std::chrono::nanoseconds>(base).count());

  // Newer message should be accepted
  EXPECT_TRUE(circular_message_cache->push(newer_message));
  // Older (out-of-order) message should be rejected by the circular buffer
  EXPECT_FALSE(circular_message_cache->push(older_message));

  circular_message_cache->notify_data_ready();
  circular_message_cache->swap_buffers();

  auto buffer = circular_message_cache->get_consumer_buffer()->data();
  // Only the newer message should be present
  EXPECT_EQ(buffer.size(), 1u);
}
