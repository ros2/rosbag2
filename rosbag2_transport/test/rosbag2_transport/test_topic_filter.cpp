// Copyright 2021, Bosch Software Innovations GmbH.
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

#include <algorithm>
#include <future>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

#include "./topic_filter.hpp"

using namespace ::testing;  // NOLINT

TEST(TestTopicFilter, filter_topics_with_more_than_one_type) {
  std::map<std::string, std::vector<std::string>> topic_with_type;
  topic_with_type.insert({"topic/a", {"type_a"}});
  topic_with_type.insert({"topic/b", {"type_b"}});
  topic_with_type.insert({"topic/c", {"type_c"}});
  {
    auto filtered_topics = rosbag2_transport::topic_filter::filter_topics_with_more_than_one_type(
      topic_with_type, true /* include hidden topics */);
    ASSERT_EQ(topic_with_type.size(), filtered_topics.size());
  }
  {
    auto filtered_topics = rosbag2_transport::topic_filter::filter_topics_with_more_than_one_type(
      topic_with_type, false /* include hidden topics */);
    ASSERT_EQ(topic_with_type.size(), filtered_topics.size());
  }

  topic_with_type.insert({"_/topic/a", {"type_a"}});
  topic_with_type.insert({"_/topic/b", {"type_b"}});
  topic_with_type.insert({"_/topic/c", {"type_c"}});
  {
    auto filtered_topics = rosbag2_transport::topic_filter::filter_topics_with_more_than_one_type(
      topic_with_type, true /* include hidden topics */);
    ASSERT_EQ(topic_with_type.size(), filtered_topics.size());
  }
  {
    auto filtered_topics = rosbag2_transport::topic_filter::filter_topics_with_more_than_one_type(
      topic_with_type, false);
    ASSERT_EQ(topic_with_type.size() - 3, filtered_topics.size());
  }

  topic_with_type.insert({"_/topic/aaa", {"type_a", "type_a", "type_a"}});
  topic_with_type.insert({"_/topic/bbb", {"type_b", "type_b", "type_b"}});
  topic_with_type.insert({"_/topic/ccc", {"type_c", "type_c", "type_c"}});
  {
    auto filtered_topics = rosbag2_transport::topic_filter::filter_topics_with_more_than_one_type(
      topic_with_type, true /* include hidden topics */);
    ASSERT_EQ(topic_with_type.size() - 3, filtered_topics.size());
    for (const auto & topic :
      {"topic/a", "topic/b", "topic/c", "_/topic/a", "_/topic/b", "_/topic/c"})
    {
      EXPECT_TRUE(filtered_topics.find(topic) != filtered_topics.end());
    }
  }
  {
    auto filtered_topics = rosbag2_transport::topic_filter::filter_topics_with_more_than_one_type(
      topic_with_type, false);
    ASSERT_EQ(topic_with_type.size() - 2 * 3, filtered_topics.size());
    for (const auto & topic : {"topic/a", "topic/b", "topic/c"}) {
      EXPECT_TRUE(filtered_topics.find(topic) != filtered_topics.end());
    }
  }
}

TEST(TestTopicFilter, filter_topics) {
  std::unordered_map<std::string, std::string> topic_with_type;
  topic_with_type.insert({"topic/a", "type_a"});
  topic_with_type.insert({"topic/b", "type_b"});
  topic_with_type.insert({"topic/c", "type_c"});

  {
    auto filtered_topics = rosbag2_transport::topic_filter::filter_topics(
      {"topic/a"},
      topic_with_type);
    ASSERT_EQ(1u, filtered_topics.size());
    ASSERT_EQ("topic/a", filtered_topics.begin()->first);
  }

  {
    auto filtered_topics = rosbag2_transport::topic_filter::filter_topics(
      {"topic/a", "topic/b",
        "topic/c"},
      topic_with_type);
    ASSERT_EQ(3u, filtered_topics.size());
    for (const auto & topic : {"topic/a", "topic/b", "topic/c"}) {
      EXPECT_TRUE(filtered_topics.find(topic) != filtered_topics.end());
    }
  }

  {
    auto filtered_topics = rosbag2_transport::topic_filter::filter_topics(
      {"topic/d", "topic/e",
        "topic/f"},
      topic_with_type);
    ASSERT_EQ(0u, filtered_topics.size());
  }

  {
    auto filtered_topics = rosbag2_transport::topic_filter::filter_topics(
      {"topic/a", "topic/b",
        "topic/d"},
      topic_with_type);
    ASSERT_EQ(2u, filtered_topics.size());
    for (const auto & topic : {"topic/a", "topic/b"}) {
      EXPECT_TRUE(filtered_topics.find(topic) != filtered_topics.end());
    }
  }
}
