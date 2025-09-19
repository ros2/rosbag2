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

#include "rcutils/time.h"
#include "rosbag2_transport/readers_manager.hpp"
#include "rosbag2_transport_test_fixture.hpp"
#include "test_msgs/message_fixtures.hpp"

using namespace ::testing;             // NOLINT
using namespace rosbag2_transport;     // NOLINT
using namespace std::chrono_literals;  // NOLINT

class Rosbag2ReadersWrapperTestFixture : public Rosbag2TransportTestFixture
{
public:
  Rosbag2ReadersWrapperTestFixture()
  : Rosbag2TransportTestFixture() {}

  ~Rosbag2ReadersWrapperTestFixture() override = default;

protected:
  void SetUp() override
  {
    // Create readers with messages at various timestamps
    auto basic_message = get_messages_basic_types()[0];
    basic_message->int32_value = 42;
    std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages1;
    std::vector<rosbag2_storage::TopicMetadata> topics1{
      {1u, "topic1", "test_msgs/msg/BasicTypes", "", {}, ""}
    };

    for (int32_t i = 0; i < kNumMessagesPerBag; i++) {
      int64_t timestamp_ms = i * 10 + kReader1MsgsOffset;  // 0, 10, 20, 30, 40 ms
      auto msg = serialize_test_message("topic1", timestamp_ms, basic_message);
      messages1.push_back(msg);
    }

    std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages2;
    std::vector<rosbag2_storage::TopicMetadata> topics2{
      {1u, "topic2", "test_msgs/msg/Strings", "", {}, ""}
    };
    auto string_message = get_messages_strings()[0];
    string_message->string_value = "Hello, world!";
    for (int32_t i = 0; i < kNumMessagesPerBag; i++) {
      int64_t timestamp_ms = i * 10 + kReader2MsgsOffset;  // 5, 15, 25, 35, 45 ms
      auto msg = serialize_test_message("topic2", timestamp_ms, string_message);
      messages2.push_back(msg);
    }

    auto reader1 = std::make_unique<MockSequentialReader>();
    reader1->prepare(messages1, topics1);
    auto cpp_reader1 = std::make_unique<rosbag2_cpp::Reader>(std::move(reader1));

    auto reader2 = std::make_unique<MockSequentialReader>();
    reader2->prepare(messages2, topics2);
    auto cpp_reader2 = std::make_unique<rosbag2_cpp::Reader>(std::move(reader2));

    readers_with_options_.emplace_back(std::move(cpp_reader1), storage_options_);
    readers_with_options_.emplace_back(std::move(cpp_reader2), storage_options_);
  }

  const int32_t kNumMessagesPerBag = 5;
  const int32_t kReader1MsgsOffset = 0;
  const int32_t kReader2MsgsOffset = 5;
  std::vector<ReadersManager::reader_storage_options_pair_t> readers_with_options_;
};

TEST_F(Rosbag2ReadersWrapperTestFixture, default_ctor_dtor)
{
  {
    std::vector<ReadersManager::reader_storage_options_pair_t> readers_with_options;
    EXPECT_THROW(
      ReadersManager readers_manager(std::move(readers_with_options)),
      std::invalid_argument
    );
  }
  {
    EXPECT_NO_THROW(ReadersManager readers_manager(std::move(readers_with_options_)));
  }
}

TEST_F(Rosbag2ReadersWrapperTestFixture, read_messages_chronologically_from_multiple_readers)
{
  ReadersManager readers_manager(std::move(readers_with_options_));

  size_t expected_total_messages = 2 * kNumMessagesPerBag;  // 5 from each reader

  for (size_t i = 0; i < expected_total_messages; ++i) {
    EXPECT_TRUE(readers_manager.has_next());
    auto message = readers_manager.get_next_message_in_chronological_order();
    ASSERT_NE(message, nullptr);
    // Expected timestamps in chronological order: 0, 5, 10, 15, 20, 25 ms etc.
    EXPECT_EQ(message->recv_timestamp, RCUTILS_MS_TO_NS((i * 5))) << "i = " << i;
    EXPECT_TRUE(
      (i % 2) ? message->topic_name == "topic2" : message->topic_name == "topic1") << "i = " << i;
  }

  // After all messages are read, has_next should return true
  EXPECT_FALSE(readers_manager.has_next());
  // And get_next_message_in_chronological_order should return nullptr
  EXPECT_EQ(readers_manager.get_next_message_in_chronological_order(), nullptr);
}

TEST_F(Rosbag2ReadersWrapperTestFixture, seek_in_multiple_readers)
{
  ReadersManager readers_manager(std::move(readers_with_options_));

  // Seek to timestamp 22ms - should get message at 25ms from reader2 as the next one
  readers_manager.seek(RCUTILS_MS_TO_NS(22));

  auto message = readers_manager.get_next_message_in_chronological_order();
  ASSERT_NE(message, nullptr);
  EXPECT_EQ(message->recv_timestamp, RCUTILS_MS_TO_NS(25));  // 25ms
  EXPECT_EQ(message->topic_name, "topic2");

  // Next message should be at 30ms from reader1
  message = readers_manager.get_next_message_in_chronological_order();
  ASSERT_NE(message, nullptr);
  EXPECT_EQ(message->recv_timestamp, RCUTILS_MS_TO_NS(30));  // 30ms
  EXPECT_EQ(message->topic_name, "topic1");
}

TEST_F(Rosbag2ReadersWrapperTestFixture, get_storage_options_from_multiple_readers)
{
  // Create different storage options for each reader
  rosbag2_storage::StorageOptions storage_options1 = {"uri1", "storage_id", 0, 100};
  rosbag2_storage::StorageOptions storage_options2 = {"uri2", "storage_id", 0, 100};

  readers_with_options_[0].second = storage_options1;
  readers_with_options_[1].second = storage_options2;

  ReadersManager readers_manager(std::move(readers_with_options_));

  // Get all storage options
  auto options = readers_manager.get_all_storage_options();
  ASSERT_EQ(options.size(), 2u);
  EXPECT_EQ(options[0].uri, "uri1");
  EXPECT_EQ(options[1].uri, "uri2");
}

TEST_F(Rosbag2ReadersWrapperTestFixture, get_earliest_and_latest_timestamps_from_multiple_readers)
{
  ReadersManager readers_manager(std::move(readers_with_options_));

  // Earliest timestamp should be 0ms (from first reader)
  EXPECT_EQ(readers_manager.get_earliest_timestamp(), 0);

  // Latest timestamp should be 45ms (from second reader)
  rcutils_time_point_value_t expected_latest_timestamp =
    RCUTILS_MS_TO_NS((kNumMessagesPerBag - 1) * 10 + kReader2MsgsOffset);
  EXPECT_EQ(readers_manager.get_latest_timestamp(), expected_latest_timestamp);
}

TEST_F(Rosbag2ReadersWrapperTestFixture, set_filter_on_multiple_readers)
{
  // Create readers with different topics
  auto basic_message = get_messages_basic_types()[0];
  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages1;
  std::vector<rosbag2_storage::TopicMetadata> topics1{
    {1u, "topic1", "test_msgs/msg/BasicTypes", "", {}, ""},
    {2u, "topic2", "test_msgs/msg/BasicTypes", "", {}, ""}
  };

  messages1.push_back(serialize_test_message("topic1", 10, basic_message));
  messages1.push_back(serialize_test_message("topic2", 20, basic_message));

  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages2;
  std::vector<rosbag2_storage::TopicMetadata> topics2{
    {1u, "topic3", "test_msgs/msg/BasicTypes", "", {}, ""},
    {2u, "topic4", "test_msgs/msg/BasicTypes", "", {}, ""}
  };

  messages2.push_back(serialize_test_message("topic3", 15, basic_message));
  messages2.push_back(serialize_test_message("topic4", 25, basic_message));

  auto mock_reader1 = std::make_unique<MockSequentialReader>();
  mock_reader1->prepare(messages1, topics1);
  auto cpp_reader1 = std::make_unique<rosbag2_cpp::Reader>(std::move(mock_reader1));

  auto mock_reader2 = std::make_unique<MockSequentialReader>();
  mock_reader2->prepare(messages2, topics2);
  auto cpp_reader2 = std::make_unique<rosbag2_cpp::Reader>(std::move(mock_reader2));

  std::vector<ReadersManager::reader_storage_options_pair_t> readers_with_options;
  readers_with_options.emplace_back(std::move(cpp_reader1), storage_options_);
  readers_with_options.emplace_back(std::move(cpp_reader2), storage_options_);

  ReadersManager readers_manager(std::move(readers_with_options));

  // Set filter to include only topic1 and topic3
  rosbag2_storage::StorageFilter filter;
  filter.topics = {"topic1", "topic3"};
  readers_manager.set_filter(filter);

  // Get all topics and types - should include all topics since filter is applied to reading
  // messages, not to the metadata
  auto topics = readers_manager.get_all_topics_and_types();
  ASSERT_EQ(topics.size(), 4u);

  // Topics might come in any order, so check for existence of each topic
  std::vector<std::string> expected_topics = {"topic1", "topic2", "topic3", "topic4"};
  for (const auto & expected_topic : expected_topics) {
    bool found = false;
    for (const auto & actual_topic : topics) {
      if (actual_topic.name == expected_topic) {
        found = true;
        break;
      }
    }
    EXPECT_TRUE(found) << "Expected topic " << expected_topic << " was not found";
  }

  while (auto message = readers_manager.get_next_message_in_chronological_order()) {
    // Only messages from topic1 and topic3 should be returned
    EXPECT_TRUE(
      message->topic_name == "topic1" || message->topic_name == "topic3")
      << "Unexpected topic: " << message->topic_name;
  }
}

TEST_F(Rosbag2ReadersWrapperTestFixture, get_all_topics_and_types_from_multiple_readers)
{
  ReadersManager readers_manager(std::move(readers_with_options_));

  // Get all topics and types
  auto topics = readers_manager.get_all_topics_and_types();

  ASSERT_EQ(topics.size(), 2u);

  // Topics might come in any order, so we need to check both possibilities
  if (topics[0].name == "topic1") {
    EXPECT_EQ(topics[0].type, "test_msgs/msg/BasicTypes");
    EXPECT_EQ(topics[1].name, "topic2");
    EXPECT_EQ(topics[1].type, "test_msgs/msg/Strings");
  } else {
    EXPECT_EQ(topics[0].name, "topic2");
    EXPECT_EQ(topics[0].type, "test_msgs/msg/Strings");
    EXPECT_EQ(topics[1].name, "topic1");
    EXPECT_EQ(topics[1].type, "test_msgs/msg/BasicTypes");
  }
}
