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
#include <unordered_set>
#include <vector>

#include "rosbag2_transport/topic_filter.hpp"

using namespace ::testing;  // NOLINT

class RegexFixture : public Test
{
protected:
  std::map<std::string, std::vector<std::string>> topics_and_types_ = {
    {"/planning", {"planning_topic_type"}},
    {"/invalid_topic", {"invalid_topic_type"}},
    {"/invalidated_topic", {"invalidated_topic_type"}},
    {"/localization", {"localization_topic_type"}},
    {"/invisible", {"invisible_topic_type"}},
    {"/status", {"status_topic_type"}}
  };
};

TEST(TestTopicFilter, filter_hidden_topics) {
  std::map<std::string, std::vector<std::string>> topics_and_types {
    {"topic/a", {"type_a"}},
    {"topic/b", {"type_b"}},
    {"topic/c", {"type_c"}},
    {"_/topic/a", {"type_a"}},
    {"_/topic/b", {"type_b"}},
    {"_/topic/c", {"type_c"}},
  };

  {
    rosbag2_transport::RecordOptions record_options;
    record_options.include_hidden_topics = true;
    rosbag2_transport::TopicFilter filter{record_options, nullptr, true};
    auto filtered_topics = filter.filter_topics(topics_and_types);
    ASSERT_EQ(topics_and_types.size(), filtered_topics.size());
  }
  {
    rosbag2_transport::RecordOptions record_options;
    record_options.include_hidden_topics = false;
    rosbag2_transport::TopicFilter filter{record_options, nullptr, true};
    auto filtered_topics = filter.filter_topics(topics_and_types);
    ASSERT_EQ(topics_and_types.size() - 3, filtered_topics.size());
  }
}

TEST(TestTopicFilter, filter_topics_with_more_than_one_type) {
  std::map<std::string, std::vector<std::string>> topics_and_types {
    {"topic/a", {"type_a", "type_a", "type_a"}},
    {"topic/b", {"type_b"}},
    {"topic/c", {"type_c", "type_c2"}},
    {"topic/d", {"type_d", "type_d", "type_d2"}},
  };

  rosbag2_transport::TopicFilter filter{rosbag2_transport::RecordOptions{}, nullptr, true};
  auto filtered_topics = filter.filter_topics(topics_and_types);
  EXPECT_THAT(filtered_topics, SizeIs(2));
  for (const auto & topic :
    {"topic/a", "topic/b"})
  {
    EXPECT_TRUE(filtered_topics.find(topic) != filtered_topics.end());
  }
}

TEST(TestTopicFilter, filter_topics_with_known_type_invalid) {
  std::map<std::string, std::vector<std::string>> topics_and_types {
    {"topic/a", {"type_a"}},
    {"topic/b", {"type_b"}},
    {"topic/c", {"type_c"}}
  };

  rosbag2_transport::TopicFilter filter{rosbag2_transport::RecordOptions{}, nullptr};
  auto filtered_topics = filter.filter_topics(topics_and_types);
  ASSERT_EQ(0u, filtered_topics.size());
}

TEST(TestTopicFilter, filter_topics_with_known_type_valid) {
  std::map<std::string, std::vector<std::string>> topics_and_types {
    {"topic/a", {"test_msgs/BasicTypes"}},
    {"topic/b", {"test_msgs/BasicTypes"}},
    {"topic/c", {"test_msgs/BasicTypes"}}
  };
  rosbag2_transport::TopicFilter filter{rosbag2_transport::RecordOptions{}, nullptr};
  auto filtered_topics = filter.filter_topics(topics_and_types);
  ASSERT_EQ(3u, filtered_topics.size());
}

TEST(TestTopicFilter, filter_topics) {
  std::map<std::string, std::vector<std::string>> topics_and_types {
    {"topic/a", {"type_a"}},
    {"topic/b", {"type_b"}},
    {"topic/c", {"type_c"}}
  };

  {
    rosbag2_transport::RecordOptions record_options;
    record_options.topics = {"topic/a"};
    rosbag2_transport::TopicFilter filter{record_options, nullptr, true};
    auto filtered_topics = filter.filter_topics(topics_and_types);
    ASSERT_EQ(1u, filtered_topics.size());
    ASSERT_EQ("topic/a", filtered_topics.begin()->first);
  }

  {
    rosbag2_transport::RecordOptions record_options;
    record_options.topics = {"topic/a", "topic/b", "topic/c"};
    rosbag2_transport::TopicFilter filter{record_options, nullptr, true};
    auto filtered_topics = filter.filter_topics(topics_and_types);
    ASSERT_EQ(3u, filtered_topics.size());
    for (const auto & topic : {"topic/a", "topic/b", "topic/c"}) {
      EXPECT_TRUE(filtered_topics.find(topic) != filtered_topics.end());
    }
  }

  {
    rosbag2_transport::RecordOptions record_options;
    record_options.topics = {"topic/d", "topic/e", "topic/f"};
    rosbag2_transport::TopicFilter filter{record_options, nullptr, true};
    auto filtered_topics = filter.filter_topics(topics_and_types);
    ASSERT_EQ(0u, filtered_topics.size());
  }

  {
    rosbag2_transport::RecordOptions record_options;
    record_options.topics = {"topic/a", "topic/b", "topic/d"};
    rosbag2_transport::TopicFilter filter{record_options, nullptr, true};
    auto filtered_topics = filter.filter_topics(topics_and_types);
    ASSERT_EQ(2u, filtered_topics.size());
    for (const auto & topic : {"topic/a", "topic/b"}) {
      EXPECT_TRUE(filtered_topics.find(topic) != filtered_topics.end());
    }
  }
}

TEST_F(RegexFixture, regex_all_and_exclude)
{
  rosbag2_transport::RecordOptions record_options;
  record_options.exclude = "/inv.*";
  record_options.all = true;
  rosbag2_transport::TopicFilter filter{record_options, nullptr, true};
  auto filtered_topics = filter.filter_topics(topics_and_types_);

  EXPECT_THAT(filtered_topics, SizeIs(3));
  for (const auto & topic : {"/planning", "/localization", "/status"}) {
    EXPECT_TRUE(filtered_topics.find(topic) != filtered_topics.end());
  }
}

TEST_F(RegexFixture, regex_filter_exclude)
{
  rosbag2_transport::RecordOptions record_options;
  record_options.regex = "/invalid.*";
  record_options.exclude = ".invalidated.*";
  record_options.all = false;
  rosbag2_transport::TopicFilter filter{record_options, nullptr, true};
  auto filtered_topics = filter.filter_topics(topics_and_types_);

  EXPECT_THAT(filtered_topics, SizeIs(1));
  EXPECT_TRUE(filtered_topics.find("/invalid_topic") != filtered_topics.end());
}

TEST_F(RegexFixture, regex_filter)
{
  rosbag2_transport::RecordOptions record_options;
  record_options.regex = "^/inval";
  record_options.all = false;
  rosbag2_transport::TopicFilter filter{record_options, nullptr, true};
  auto filtered_topics = filter.filter_topics(topics_and_types_);

  EXPECT_THAT(filtered_topics, SizeIs(2));
  for (const auto & topic : {"/invalid_topic", "/invalidated_topic"}) {
    EXPECT_TRUE(filtered_topics.find(topic) != filtered_topics.end());
  }
}

TEST_F(RegexFixture, regex_all_and_filter)
{
  rosbag2_transport::RecordOptions record_options;
  record_options.regex = "/status";
  record_options.all = true;
  rosbag2_transport::TopicFilter filter{record_options, nullptr, true};
  auto filtered_topics = filter.filter_topics(topics_and_types_);
  EXPECT_THAT(filtered_topics, SizeIs(6));
}
