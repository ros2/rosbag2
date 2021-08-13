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

#include "./topic_filter.hpp"

using namespace ::testing;  // NOLINT

class RegexFixture : public Test
{
protected:
  std::unordered_map<std::string, std::string> topics_and_types_ = {
    {"/planning", "planning_topic_type"},
    {"/invalid_topic", "invalid_topic_type"},
    {"/invalidated_topic", "invalidated_topic_type"},
    {"/localization", "localization_topic_type"},
    {"/invisible", "invisible_topic_type"},
    {"/status", "status_topic_type"}
  };
};

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

TEST(TestTopicFilter, filter_topics_with_known_type) {
  std::unordered_map<std::string, std::string> topic_with_type;
  std::unordered_set<std::string> topic_unknown_types;
  topic_with_type.insert({"topic/a", "type_a"});
  topic_with_type.insert({"topic/b", "type_b"});
  topic_with_type.insert({"topic/c", "type_c"});
  {
    auto filtered_topics = rosbag2_transport::topic_filter::filter_topics_with_known_type(
      topic_with_type, topic_unknown_types);
    ASSERT_EQ(0u, filtered_topics.size());
  }
  topic_with_type.clear();
  topic_with_type.insert({"topic/a", "test_msgs/BasicTypes"});
  topic_with_type.insert({"topic/b", "test_msgs/BasicTypes"});
  topic_with_type.insert({"topic/c", "test_msgs/BasicTypes"});
  {
    auto filtered_topics = rosbag2_transport::topic_filter::filter_topics_with_known_type(
      topic_with_type, topic_unknown_types);
    ASSERT_EQ(3u, filtered_topics.size());
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

TEST_F(RegexFixture, regex_all_and_exclude)
{
  std::string filter_regex_string = "";
  std::string exclude_regex_string = "/inv.*";
  bool all_flag = true;

  auto filtered_topics = rosbag2_transport::topic_filter::filter_topics_using_regex(
    topics_and_types_,
    filter_regex_string,
    exclude_regex_string,
    all_flag
  );

  EXPECT_THAT(filtered_topics, SizeIs(3));
  for (const auto & topic : {"/planning", "/localization", "/status"}) {
    EXPECT_TRUE(filtered_topics.find(topic) != filtered_topics.end());
  }
}

TEST_F(RegexFixture, regex_filter_exclude)
{
  std::string filter_regex_string = "/invalid.*";
  std::string exclude_regex_string = ".invalidated.*";
  bool all_flag = false;

  auto filtered_topics = rosbag2_transport::topic_filter::filter_topics_using_regex(
    topics_and_types_,
    filter_regex_string,
    exclude_regex_string,
    all_flag
  );

  EXPECT_THAT(filtered_topics, SizeIs(1));
  EXPECT_TRUE(filtered_topics.find("/invalid_topic") != filtered_topics.end());
}

TEST_F(RegexFixture, regex_filter)
{
  std::string filter_regex_string = "/inval.*";
  std::string exclude_regex_string = "";
  bool all_flag = false;

  auto filtered_topics = rosbag2_transport::topic_filter::filter_topics_using_regex(
    topics_and_types_,
    filter_regex_string,
    exclude_regex_string,
    all_flag
  );

  EXPECT_THAT(filtered_topics, SizeIs(2));
  for (const auto & topic : {"/invalid_topic", "/invalidated_topic"}) {
    EXPECT_TRUE(filtered_topics.find(topic) != filtered_topics.end());
  }
}

TEST_F(RegexFixture, regex_all_and_filter)
{
  std::string filter_regex_string = "/status";
  std::string exclude_regex_string = "";
  bool all_flag = true;

  auto filtered_topics = rosbag2_transport::topic_filter::filter_topics_using_regex(
    topics_and_types_,
    filter_regex_string,
    exclude_regex_string,
    all_flag
  );

  EXPECT_THAT(filtered_topics, SizeIs(6));
}
