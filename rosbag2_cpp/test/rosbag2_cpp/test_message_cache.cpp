// Copyright 2020, Robotec.ai sp. z o.o.
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
#include <numeric>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <thread>

#include "rcutils/time.h"
#include "rosbag2_cpp/cache/message_cache_buffer.hpp"
#include "rosbag2_storage/ros_helper.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"

#include "mock_cache_consumer.hpp"
#include "mock_message_cache.hpp"

using namespace testing;  // NOLINT
using namespace std::chrono_literals;

namespace
{
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

uint32_t sum_up(const std::unordered_map<std::string, uint32_t> & map)
{
  return std::accumulate(
    std::begin(map),
    std::end(map),
    0,
    [](const uint32_t previous, const auto & element) {
      return previous + element.second;
    }
  );
}
}  // namespace

class MessageCacheTest : public Test
{
public:
  MessageCacheTest() = default;

  virtual ~MessageCacheTest() = default;

  const uint64_t cache_size_ {1 * 1000};  // ~1 Kb cache
};

TEST_F(MessageCacheTest, message_cache_writes_full_producer_buffer) {
  const uint32_t message_count = 100;
  uint64_t size_bytes_so_far = 0;
  size_t should_be_dropped_count = 0;
  size_t consumed_message_count {0};

  auto mock_message_cache = std::make_shared<NiceMock<MockMessageCache>>(
    cache_size_);

  for (uint32_t i = 0; i < message_count; ++i) {
    auto msg = make_test_msg();
    size_t serialized_data_size = msg->serialized_data->buffer_length;
    mock_message_cache->push(msg);
    if (cache_size_ < size_bytes_so_far) {
      should_be_dropped_count++;
    }
    size_bytes_so_far += serialized_data_size;
  }

  auto total_actually_dropped = sum_up(mock_message_cache->messages_dropped());
  EXPECT_EQ(should_be_dropped_count, total_actually_dropped);

  auto cb = [&consumed_message_count](
    const std::vector<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>> & msgs) {
      consumed_message_count += msgs.size();
    };

  auto mock_cache_consumer = std::make_unique<NiceMock<MockCacheConsumer>>(
    mock_message_cache,
    cb);

  using namespace std::chrono_literals;
  std::this_thread::sleep_for(20ms);

  mock_cache_consumer->stop();
  EXPECT_EQ(consumed_message_count, message_count - should_be_dropped_count);
}

TEST_F(MessageCacheTest, message_cache_rejects_null_message) {
  auto message_cache = std::make_shared<rosbag2_cpp::cache::MessageCache>(500);

  bool result = true;
  ASSERT_NO_THROW(result = message_cache->push(nullptr));
  EXPECT_FALSE(result);

  // Test message with null serialized data
  auto msg = make_test_msg();
  msg->serialized_data = nullptr;
  ASSERT_NO_THROW(result = message_cache->push(msg));
  EXPECT_FALSE(result);
}

TEST_F(MessageCacheTest, constructor_throws_if_both_limits_are_zero) {
  EXPECT_THROW(rosbag2_cpp::cache::MessageCache(0, 0), std::invalid_argument);
}

TEST_F(MessageCacheTest, message_cache_buffer_time_only_limits_by_duration) {
  // Buffer with time bound only: 2 seconds, size unlimited (0)
  constexpr uint32_t max_duration_sec = 2;
  constexpr size_t max_buffer_size = 0;

  auto message_cache = rosbag2_cpp::cache::MessageCache(max_buffer_size, max_duration_sec);

  const auto base = 10s;
  const auto messages_interval = 500ms;
  constexpr size_t total_message_count = 10;  // 5 seconds total
  // Expected 5 messages in buffer
  const size_t expected_msgs = std::chrono::seconds(max_duration_sec) / messages_interval + 1;
  ASSERT_LT(expected_msgs, total_message_count);
  for (size_t i = 0; i < expected_msgs; i++) {
    const auto ts = base + (i * messages_interval);
    auto msg = make_test_msg(std::chrono::duration_cast<std::chrono::nanoseconds>(ts).count());
    ASSERT_TRUE(message_cache.push(msg));
  }
  const size_t drop_msgs_start_index = total_message_count - expected_msgs;
  for (size_t i = drop_msgs_start_index; i < total_message_count; i++) {
    const auto ts = base + (i * messages_interval);
    auto msg = make_test_msg(std::chrono::duration_cast<std::chrono::nanoseconds>(ts).count());
    EXPECT_FALSE(message_cache.push(msg));
  }

  message_cache.notify_data_ready();
  message_cache.swap_buffers();
  auto consumer_buffer = message_cache.get_consumer_buffer();
  const auto & buffer_data = consumer_buffer->data();
  message_cache.release_consumer_buffer();

  ASSERT_EQ(buffer_data.size(), expected_msgs);
  const auto oldest = buffer_data.front()->recv_timestamp;
  const auto newest = buffer_data.back()->recv_timestamp;
  const auto span_ns = newest - oldest;
  EXPECT_GT(span_ns, 0);
  EXPECT_LE(span_ns, RCUTILS_S_TO_NS(max_duration_sec));
}

TEST_F(MessageCacheTest, message_cache_buffer_size_only_limits_by_size) {
  // Buffer with size bound only: 300 bytes, time unlimited (0)
  constexpr size_t max_buffer_size = 300;
  constexpr uint32_t max_duration_sec = 0;

  auto message_cache = rosbag2_cpp::cache::MessageCache(max_buffer_size, max_duration_sec);

  // Push messages until size limit triggers drop behavior
  const auto base = 1s;
  const auto messages_interval = 100ms;
  constexpr size_t total_message_count = 50;  // 5 seconds total
  for (size_t i = 0; i < total_message_count; i++) {
    const auto ts = base + (i * messages_interval);
    const auto ts_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(ts).count();
    auto msg = make_test_msg(ts_ns, std::string(17, 'x'));  // ~1 bytes payload. 850 bytes total
    (void)message_cache.push(msg);
  }

  message_cache.notify_data_ready();
  message_cache.swap_buffers();
  auto consumer_buffer = message_cache.get_consumer_buffer();
  const auto & buffer_data = consumer_buffer->data();
  message_cache.release_consumer_buffer();

  ASSERT_GT(buffer_data.size(), 0u);
  size_t total_size = 0;
  for (const auto & m : buffer_data) {
    total_size += m->serialized_data->buffer_length;
  }
  EXPECT_LE(total_size, max_buffer_size);
}

TEST_F(MessageCacheTest, message_cache_buffer_time_and_size_bounds_respected) {
  // Both bounds: 200 bytes and 3 seconds
  constexpr size_t max_buffer_size = 200;
  constexpr uint32_t max_duration_sec = 3;

  auto message_cache = rosbag2_cpp::cache::MessageCache(max_buffer_size, max_duration_sec);

  const auto base = 1s;
  const auto messages_interval = 100ms;
  constexpr size_t total_message_count = 50;  // 5 seconds total
  for (size_t i = 0; i < total_message_count; i++) {
    const auto ts = base + (i * messages_interval);
    const auto ts_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(ts).count();
    auto msg = make_test_msg(ts_ns, std::string(17, 'x'));  // ~17 bytes payload. 850 bytes total
    (void)message_cache.push(msg);
  }

  message_cache.notify_data_ready();
  message_cache.swap_buffers();
  auto consumer_buffer = message_cache.get_consumer_buffer();
  const auto & buffer_data = consumer_buffer->data();
  message_cache.release_consumer_buffer();

  ASSERT_GT(buffer_data.size(), 0u);
  // Check size bound
  size_t total_size = 0;
  for (const auto & m : buffer_data) {
    total_size += m->serialized_data->buffer_length;
  }
  EXPECT_LE(total_size, max_buffer_size);

  // Check time bound
  const auto oldest = buffer_data.front()->recv_timestamp;
  const auto newest = buffer_data.back()->recv_timestamp;
  const auto span_ns = newest - oldest;
  EXPECT_GT(span_ns, 0);
  EXPECT_LE(span_ns, RCUTILS_S_TO_NS(max_duration_sec));
}

TEST_F(MessageCacheTest, message_cache_allows_first_message_exceeding_size_limit) {
  // MessageCacheBuffer allows at least one message even if it exceeds max size
  constexpr size_t max_buffer_size = 200;  // 200 bytes
  constexpr uint32_t max_duration_sec = 0;  // size-only bound

  auto message_cache = rosbag2_cpp::cache::MessageCache(max_buffer_size, max_duration_sec);

  // First oversized message should be accepted
  const auto ts1_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(1s).count();
  auto big_msg = make_test_msg(ts1_ns, std::string(500, 'x'));  // ~500 bytes payload
  ASSERT_TRUE(message_cache.push(big_msg));

  // Subsequent messages should be dropped once buffer is marked to drop (size exceeded)
  const auto ts2_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(1500ms).count();
  auto another_msg = make_test_msg(ts2_ns, std::string(10, 'y'));  // small payload
  EXPECT_FALSE(message_cache.push(another_msg));

  // Consume buffer and verify only the first oversized message is present
  message_cache.notify_data_ready();
  message_cache.swap_buffers();
  auto consumer_buffer = message_cache.get_consumer_buffer();
  const auto & buffer_data = consumer_buffer->data();
  message_cache.release_consumer_buffer();

  ASSERT_EQ(buffer_data.size(), 1u);
  size_t total_size = 0;
  for (const auto & m : buffer_data) {
    total_size += m->serialized_data->buffer_length;
  }
  // The stored data size can exceed the configured max size by a single message
  EXPECT_GT(total_size, max_buffer_size);
  EXPECT_EQ(buffer_data.front()->recv_timestamp, ts1_ns);
}
