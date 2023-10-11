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
    {"/status", {"status_topic_type"}},
    {"/invalid_service/_service_event", {"service/srv/invalid_service_Event"}},
    {"/invalidated_service/_service_event", {"service/srv/invalidated_service_Event"}},
    {"/planning_service/_service_event", {"service/srv/planning_service_Event"}}
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
    record_options.all_topics = true;
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
  rosbag2_transport::RecordOptions record_options;
  record_options.all_topics = true;
  rosbag2_transport::TopicFilter filter{record_options, nullptr, true};
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
  rosbag2_transport::RecordOptions record_options;
  record_options.all_topics = true;
  rosbag2_transport::TopicFilter filter{record_options, nullptr};
  auto filtered_topics = filter.filter_topics(topics_and_types);
  ASSERT_EQ(0u, filtered_topics.size());
}

TEST(TestTopicFilter, filter_topics_with_known_type_valid) {
  std::map<std::string, std::vector<std::string>> topics_and_types {
    {"topic/a", {"test_msgs/BasicTypes"}},
    {"topic/b", {"test_msgs/BasicTypes"}},
    {"topic/c", {"test_msgs/BasicTypes"}}
  };
  rosbag2_transport::RecordOptions record_options;
  record_options.all_topics = true;
  rosbag2_transport::TopicFilter filter{record_options, nullptr};
  auto filtered_topics = filter.filter_topics(topics_and_types);
  ASSERT_EQ(3u, filtered_topics.size());
}

TEST(TestTopicFilter, filter_topics) {
  std::map<std::string, std::vector<std::string>> topics_and_types {
    {"topic/a", {"type_a"}},
    {"topic/b", {"type_b"}},
    {"topic/c", {"type_c"}},
    {"/service/a/_service_event", {"service/srv/type_a_Event"}},
    {"/service/b/_service_event", {"service/srv/type_b_Event"}},
    {"/service/c/_service_event", {"service/srv/type_c_Event"}},
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

  {
    rosbag2_transport::RecordOptions record_options;
    record_options.services = {"/service/a/_service_event"};
    rosbag2_transport::TopicFilter filter{record_options, nullptr, true};
    auto filtered_topics = filter.filter_topics(topics_and_types);
    ASSERT_EQ(1u, filtered_topics.size());
    EXPECT_EQ("/service/a/_service_event", filtered_topics.begin()->first);
  }

  {
    rosbag2_transport::RecordOptions record_options;
    record_options.services = {
      "/service/a/_service_event",
      "/service/b/_service_event",
      "/service/d/_service_event"};
    rosbag2_transport::TopicFilter filter{record_options, nullptr, true};
    auto filtered_topics = filter.filter_topics(topics_and_types);
    ASSERT_EQ(2u, filtered_topics.size());
    for (const auto & topic :
      {"/service/a/_service_event", "/service/b/_service_event"})
    {
      EXPECT_TRUE(filtered_topics.find(topic) != filtered_topics.end());
    }
  }
}

TEST_F(RegexFixture, regex_all_topics_and_exclude_topics)
{
  rosbag2_transport::RecordOptions record_options;
  record_options.exclude_topics = "/inv.*";
  record_options.all_topics = true;
  rosbag2_transport::TopicFilter filter{record_options, nullptr, true};
  auto filtered_topics = filter.filter_topics(topics_and_types_);

  EXPECT_THAT(filtered_topics, SizeIs(3));
  for (const auto & topic : {"/planning", "/localization", "/status"}) {
    EXPECT_TRUE(filtered_topics.find(topic) != filtered_topics.end());
  }
}

TEST_F(RegexFixture, regex_all_services_and_exclude_services)
{
  rosbag2_transport::RecordOptions record_options;
  record_options.exclude_services = "/inv.*";
  record_options.all_services = true;
  rosbag2_transport::TopicFilter filter{record_options, nullptr, true};
  auto filtered_topics = filter.filter_topics(topics_and_types_);

  EXPECT_THAT(filtered_topics, SizeIs(1));
  EXPECT_EQ("/planning_service/_service_event", filtered_topics.begin()->first);
}

TEST_F(RegexFixture, regex_all_topics_all_services_and_exclude_topics_and_services)
{
  rosbag2_transport::RecordOptions record_options;
  record_options.exclude_topics = "/inv.*";
  record_options.all_topics = true;
  record_options.exclude_services = "/inv.*";
  record_options.all_services = true;
  rosbag2_transport::TopicFilter filter{record_options, nullptr, true};
  auto filtered_topics = filter.filter_topics(topics_and_types_);

  EXPECT_THAT(filtered_topics, SizeIs(4));
  for (const auto & topic :
    {"/planning", "/localization", "/status", "/planning_service/_service_event"})
  {
    EXPECT_TRUE(filtered_topics.find(topic) != filtered_topics.end());
  }
}

TEST_F(RegexFixture, regex_filter_exclude_topics)
{
  rosbag2_transport::RecordOptions record_options;
  record_options.regex = "/invalid.*";
  record_options.exclude_topics = ".invalidated.*";  // Only affect topics
  rosbag2_transport::TopicFilter filter{record_options, nullptr, true};
  auto filtered_topics = filter.filter_topics(topics_and_types_);

  EXPECT_THAT(filtered_topics, SizeIs(3));
  EXPECT_TRUE(filtered_topics.find("/invalid_topic") != filtered_topics.end());
  EXPECT_TRUE(filtered_topics.find("/invalid_service/_service_event") != filtered_topics.end());
  EXPECT_TRUE(filtered_topics.find("/invalidated_service/_service_event") != filtered_topics.end());
}

TEST_F(RegexFixture, regex_filter_exclude_services)
{
  rosbag2_transport::RecordOptions record_options;
  record_options.regex = "/invalid.*";
  record_options.exclude_services = ".invalidated.*";  // Only affect services
  rosbag2_transport::TopicFilter filter{record_options, nullptr, true};
  auto filtered_topics = filter.filter_topics(topics_and_types_);

  EXPECT_THAT(filtered_topics, SizeIs(3));
  EXPECT_TRUE(filtered_topics.find("/invalid_topic") != filtered_topics.end());
  EXPECT_TRUE(filtered_topics.find("/invalidated_topic") != filtered_topics.end());
  EXPECT_TRUE(filtered_topics.find("/invalid_service/_service_event") != filtered_topics.end());
}

TEST_F(RegexFixture, regex_filter_exclude_topics_and_services)
{
  rosbag2_transport::RecordOptions record_options;
  record_options.regex = "/invalid.*";
  record_options.exclude_topics = ".invalidated.*";
  record_options.exclude_services = ".invalidated.*";
  rosbag2_transport::TopicFilter filter{record_options, nullptr, true};
  auto filtered_topics = filter.filter_topics(topics_and_types_);

  EXPECT_THAT(filtered_topics, SizeIs(2));
  EXPECT_TRUE(filtered_topics.find("/invalid_topic") != filtered_topics.end());
  EXPECT_TRUE(filtered_topics.find("/invalid_service/_service_event") != filtered_topics.end());
}

TEST_F(RegexFixture, regex_filter)
{
  rosbag2_transport::RecordOptions record_options;
  record_options.regex = "^/inval";
  rosbag2_transport::TopicFilter filter{record_options, nullptr, true};
  auto filtered_topics = filter.filter_topics(topics_and_types_);

  EXPECT_THAT(filtered_topics, SizeIs(4));
  for (const auto & topic :
    {"/invalid_topic", "/invalidated_topic", "/invalid_service/_service_event",
      "/invalidated_service/_service_event"})
  {
    EXPECT_TRUE(filtered_topics.find(topic) != filtered_topics.end());
  }
}

TEST_F(RegexFixture, regex_all_topics_and_filter)
{
  rosbag2_transport::RecordOptions record_options;
  record_options.regex = "/status";
  record_options.all_topics = true;
  record_options.exclude_services = ".*";  //  Not include services
  rosbag2_transport::TopicFilter filter{record_options, nullptr, true};
  auto filtered_topics = filter.filter_topics(topics_and_types_);
  EXPECT_THAT(filtered_topics, SizeIs(6));
}

TEST_F(RegexFixture, do_not_print_warning_about_unknown_types_if_topic_is_not_selected) {
  {  // Check for topics explicitly selected via "topics" list
    rosbag2_transport::RecordOptions record_options;
    // Select only one topic with name "/planning" via topic list
    record_options.topics = {"/planning"};
    record_options.all_topics = false;
    rosbag2_transport::TopicFilter filter{record_options, nullptr, false};
    testing::internal::CaptureStderr();
    auto filtered_topics = filter.filter_topics(topics_and_types_);
    std::string test_output = testing::internal::GetCapturedStderr();
    ASSERT_EQ(0u, filtered_topics.size());
    EXPECT_TRUE(
      test_output.find(
        "Topic '/invalid_topic' has unknown type 'invalid_topic_type'") == std::string::npos);
    EXPECT_TRUE(
      test_output.find(
        "Topic '/planning' has unknown type 'planning_topic_type'") != std::string::npos);
  }

  {  // Check for topics selected via regex
    rosbag2_transport::RecordOptions record_options;
    // Select only one topic with name "/planning" via regex
    record_options.regex = "^/planning";
    record_options.all_topics = false;
    rosbag2_transport::TopicFilter filter{record_options, nullptr, false};
    testing::internal::CaptureStderr();
    auto filtered_topics = filter.filter_topics(topics_and_types_);
    std::string test_output = testing::internal::GetCapturedStderr();
    ASSERT_EQ(0u, filtered_topics.size());
    EXPECT_TRUE(
      test_output.find(
        "Topic '/invalid_topic' has unknown type 'invalid_topic_type'") == std::string::npos);
    EXPECT_TRUE(
      test_output.find(
        "Topic '/planning' has unknown type 'planning_topic_type'") != std::string::npos);
  }
}

TEST_F(RegexFixture, regex_all_services_and_filter)
{
  rosbag2_transport::RecordOptions record_options;
  record_options.regex = "/status";
  record_options.all_services = true;
  record_options.exclude_topics = ".*";  // Not include topics
  rosbag2_transport::TopicFilter filter{record_options, nullptr, true};
  auto filtered_topics = filter.filter_topics(topics_and_types_);
  EXPECT_THAT(filtered_topics, SizeIs(3));
}

TEST_F(RegexFixture, regex_all_topics_all_services_and_filter)
{
  rosbag2_transport::RecordOptions record_options;
  record_options.regex = "/status";
  record_options.all_topics = true;
  record_options.all_services = true;
  rosbag2_transport::TopicFilter filter{record_options, nullptr, true};
  auto filtered_topics = filter.filter_topics(topics_and_types_);
  EXPECT_THAT(filtered_topics, SizeIs(9));
}
