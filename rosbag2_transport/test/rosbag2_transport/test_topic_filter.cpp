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

#include "rosbag2_cpp/action_utils.hpp"
#include "rosbag2_transport/topic_filter.hpp"

using namespace ::testing;  // NOLINT

class TestTopicFilter : public Test
{
protected:
  std::map<std::string, std::vector<std::string>> topics_services_actions_with_types_ = {
    {"/planning1", {"planning_topic_type"}},
    {"/planning2", {"planning_topic_type"}},
    {"/invalid_topic", {"invalid_topic_type"}},
    {"/invalidated_topic", {"invalidated_topic_type"}},
    {"/localization", {"localization_topic_type"}},
    {"/invisible", {"invisible_topic_type"}},
    {"/status", {"status_topic_type"}},
    // For services
    {"/invalid_service/_service_event", {"service/srv/invalid_service_Event"}},
    {"/invalidated_service/_service_event", {"service/srv/invalidated_service_Event"}},
    {"/planning_service/_service_event", {"service/srv/planning_service_Event"}},
    // invalid_action
    {"/invalid_action/_action/send_goal/_service_event",
      {"test_msgs/action/Invalid_SendGoal_Event"}},
    {"/invalid_action/_action/get_result/_service_event",
      {"test_msgs/action/Invalid_GetResult_Event"}},
    {"/invalid_action/_action/cancel_goal/_service_event", {"action_msgs/srv/CancelGoal_Event"}},
    {"/invalid_action/_action/feedback", {"test_msgs/action/Invalid_FeedbackMessage"}},
    {"/invalid_action/_action/status", {"action_msgs/msg/GoalStatusArray"}},
    // invalidated_action
    {"/invalidated_action/_action/send_goal/_service_event",
      {"test_msgs/action/Invalidated_SendGoal_Event"}},
    {"/invalidated_action/_action/get_result/_service_event",
      {"test_msgs/action/Invalidated_GetResult_Event"}},
    {"/invalidated_action/_action/cancel_goal/_service_event",
      {"action_msgs/srv/CancelGoal_Event"}},
    {"/invalidated_action/_action/feedback", {"test_msgs/action/Invalidated_FeedbackMessage"}},
    {"/invalidated_action/_action/status", {"action_msgs/msg/GoalStatusArray"}},
    // planning_action
    {"/planning_action/_action/send_goal/_service_event",
      {"unknown_pkg/action/Planning_SendGoal_Event"}},
    {"/planning_action/_action/get_result/_service_event",
      {"unknown_pkg/action/Planning_GetResult_Event"}},
    {"/planning_action/_action/cancel_goal/_service_event", {"action_msgs/srv/CancelGoal_Event"}},
    {"/planning_action/_action/feedback", {"unknown_pkg/action/Planning_FeedbackMessage"}},
    {"/planning_action/_action/status", {"action_msgs/msg/GoalStatusArray"}},
  };

  void check_action_interfaces_exist(
    std::unordered_map<std::string, std::string> & filtered_topics,
    const std::string action_name)
  {
    EXPECT_TRUE(filtered_topics.find(action_name + "/_action/send_goal/_service_event") !=
      filtered_topics.end());
    EXPECT_TRUE(filtered_topics.find(action_name + "/_action/get_result/_service_event") !=
      filtered_topics.end());
    EXPECT_TRUE(filtered_topics.find(action_name + "/_action/cancel_goal/_service_event") !=
      filtered_topics.end());
    EXPECT_TRUE(filtered_topics.find(action_name + "/_action/feedback") != filtered_topics.end());
    EXPECT_TRUE(filtered_topics.find(action_name + "/_action/status") != filtered_topics.end());
  }
};

TEST_F(TestTopicFilter, filter_hidden_topics) {
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
    record_options.all_topics = true;
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

TEST_F(TestTopicFilter, filter_topics_with_more_than_one_type) {
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

TEST_F(TestTopicFilter, filter_topics_with_known_type_invalid) {
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

TEST_F(TestTopicFilter, filter_topics_with_known_type_valid) {
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

TEST_F(TestTopicFilter, filter_topics) {
  std::map<std::string, std::vector<std::string>> topics_and_types {
    {"topic/a", {"type_a"}},
    {"topic/b", {"type_b"}},
    {"topic/c", {"type_c"}},
    {"/service/a/_service_event", {"service/srv/type_a_Event"}},
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
      EXPECT_TRUE(filtered_topics.find(topic) != filtered_topics.end()) <<
        "Expected topic:" << topic;
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
      EXPECT_TRUE(filtered_topics.find(topic) != filtered_topics.end()) <<
        "Expected topic:" << topic;
    }
  }
}

TEST_F(TestTopicFilter, filter_services) {
  std::map<std::string, std::vector<std::string>> topics_and_types{
    {"topic/a", {"type_a"}},
    {"/service/a/_service_event", {"service/srv/type_a_Event"}},
    {"/service/b/_service_event", {"service/srv/type_b_Event"}},
    {"/service/c/_service_event", {"service/srv/type_c_Event"}},
  };

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
      EXPECT_TRUE(filtered_topics.find(topic) != filtered_topics.end()) <<
        "Expected topic:" << topic;
    }
  }
}

TEST_F(TestTopicFilter, filter_actions) {
  std::map<std::string, std::vector<std::string>> topics_and_types{
    {"topic/a", {"type_a"}},
    {"/service/a/_service_event", {"service/srv/type_a_Event"}},
    // action/a
    {"/action/a/_action/send_goal/_service_event", {"test_msgs/action/TypeA_SendGoal_Event"}},
    {"/action/a/_action/get_result/_service_event", {"test_msgs/action/TypeA_GetResult_Event"}},
    {"/action/a/_action/cancel_goal/_service_event", {"action_msgs/srv/CancelGoal_Event"}},
    {"/action/a/_action/feedback", {"test_msgs/action/TypeA_FeedbackMessage"}},
    {"/action/a/_action/status", {"action_msgs/msg/GoalStatusArray"}},
    // action/b
    {"/action/b/_action/send_goal/_service_event", {"test_msgs/action/TypeB_SendGoal_Event"}},
    {"/action/b/_action/get_result/_service_event", {"test_msgs/action/TypeB_GetResult_Event"}},
    {"/action/b/_action/cancel_goal/_service_event", {"action_msgs/srv/CancelGoal_Event"}},
    {"/action/b/_action/feedback", {"test_msgs/action/TypeB_FeedbackMessage"}},
    {"/action/b/_action/status", {"action_msgs/msg/GoalStatusArray"}},
    // action/c
    {"/action/c/_action/send_goal/_service_event", {"test_msgs/action/TypeC_SendGoal_Event"}},
    {"/action/c/_action/get_result/_service_event", {"test_msgs/action/TypeC_GetResult_Event"}},
    {"/action/c/_action/cancel_goal/_service_event", {"action_msgs/srv/CancelGoal_Event"}},
    {"/action/c/_action/feedback", {"test_msgs/action/TypeC_FeedbackMessage"}},
    {"/action/c/_action/status", {"action_msgs/msg/GoalStatusArray"}},
  };

  {
    rosbag2_transport::RecordOptions record_options;

    record_options.actions = {
      "/action/a"
    };
    rosbag2_transport::TopicFilter filter{record_options, nullptr, true};
    auto filtered_topics = filter.filter_topics(topics_and_types);
    ASSERT_EQ(5u, filtered_topics.size());
    auto expected_action_interface_names =
      rosbag2_cpp::action_name_to_action_interface_names(record_options.actions[0]);
    for (auto & topic : expected_action_interface_names) {
      EXPECT_TRUE(filtered_topics.find(topic) != filtered_topics.end()) <<
        "Expected topic:" << topic;
    }
  }

  {
    rosbag2_transport::RecordOptions record_options;
    record_options.actions = {
      "/action/a",
      "/action/b",
      "/action/d"
    };
    rosbag2_transport::TopicFilter filter{record_options, nullptr, true};
    auto filtered_topics = filter.filter_topics(topics_and_types);

    // all action interface names for /action/a and /action/b
    std::vector<std::string> expected_action_interfaces_name;
    auto expected_action_a_interface_names =
      rosbag2_cpp::action_name_to_action_interface_names(record_options.actions[0]);
    auto expected_action_b_interface_names =
      rosbag2_cpp::action_name_to_action_interface_names(record_options.actions[1]);
    std::copy(expected_action_a_interface_names.begin(),
      expected_action_a_interface_names.end(),
      std::back_inserter(expected_action_interfaces_name));
    std::copy(expected_action_b_interface_names.begin(),
      expected_action_b_interface_names.end(),
      std::back_inserter(expected_action_interfaces_name));

    ASSERT_EQ(10u, filtered_topics.size());

    for (auto & topic : expected_action_interfaces_name) {
      EXPECT_TRUE(filtered_topics.find(topic) != filtered_topics.end()) <<
        "Expected topic:" << topic;
    }
  }
}

TEST_F(TestTopicFilter, all_topics_and_exclude_regex)
{
  rosbag2_transport::RecordOptions record_options;
  record_options.exclude_regex = "/inv.*";
  record_options.all_topics = true;
  rosbag2_transport::TopicFilter filter{record_options, nullptr, true};
  auto filtered_topics = filter.filter_topics(topics_services_actions_with_types_);

  EXPECT_THAT(filtered_topics, SizeIs(4));
  for (const auto & topic : {"/planning1", "/planning2", "/localization", "/status"}) {
    EXPECT_TRUE(filtered_topics.find(topic) != filtered_topics.end()) << "Expected topic:" << topic;
  }
}

TEST_F(TestTopicFilter, all_topics_and_exclude_topics)
{
  rosbag2_transport::RecordOptions record_options;
  record_options.exclude_topics = {
    "/invalid_topic",
    "/invalidated_topic",
    "/invisible"};
  record_options.all_topics = true;
  rosbag2_transport::TopicFilter filter{record_options, nullptr, true};
  auto filtered_topics = filter.filter_topics(topics_services_actions_with_types_);

  EXPECT_THAT(filtered_topics, SizeIs(4));
  for (const auto & topic : {"/planning1", "/planning2", "/localization", "/status"}) {
    EXPECT_TRUE(filtered_topics.find(topic) != filtered_topics.end()) << "Expected topic:" << topic;
  }
}

TEST_F(TestTopicFilter, all_topics_and_exclude_type_topics)
{
  rosbag2_transport::RecordOptions record_options;
  record_options.exclude_topic_types = {
    "localization_topic_type",
    "status_topic_type"};
  record_options.all_topics = true;
  rosbag2_transport::TopicFilter filter{record_options, nullptr, true};
  auto filtered_topics = filter.filter_topics(topics_services_actions_with_types_);

  EXPECT_THAT(filtered_topics, SizeIs(5));
  for (const auto & topic :
    {"/planning1", "/planning2", "/invisible", "/invalidated_topic", "/invalid_topic"})
  {
    EXPECT_TRUE(filtered_topics.find(topic) != filtered_topics.end()) << "Expected topic:" << topic;
  }

  EXPECT_TRUE(filtered_topics.find("/localization") == filtered_topics.end());
  EXPECT_TRUE(filtered_topics.find("/status") == filtered_topics.end());
}

TEST_F(TestTopicFilter, all_services_and_exclude_regex)
{
  rosbag2_transport::RecordOptions record_options;
  record_options.exclude_regex = "/inv.*";
  record_options.all_topics = false;
  record_options.all_services = true;
  record_options.all_actions = false;
  rosbag2_transport::TopicFilter filter{record_options, nullptr, true};
  auto filtered_topics = filter.filter_topics(topics_services_actions_with_types_);

  EXPECT_THAT(filtered_topics, SizeIs(1));
  EXPECT_EQ("/planning_service/_service_event", filtered_topics.begin()->first);
}

TEST_F(TestTopicFilter, all_services_and_exclude_service_events)
{
  rosbag2_transport::RecordOptions record_options;
  record_options.exclude_service_events = {
    "/invalid_service/_service_event",
    "/invalidated_service/_service_event"
  };
  record_options.all_services = true;
  rosbag2_transport::TopicFilter filter{record_options, nullptr, true};
  auto filtered_topics = filter.filter_topics(topics_services_actions_with_types_);

  EXPECT_THAT(filtered_topics, SizeIs(1));
  EXPECT_EQ("/planning_service/_service_event", filtered_topics.begin()->first);
}

TEST_F(TestTopicFilter, all_topics_all_services_and_exclude_regex)
{
  rosbag2_transport::RecordOptions record_options;
  record_options.all_topics = true;
  record_options.all_services = true;
  record_options.exclude_regex = "/inv.*";
  rosbag2_transport::TopicFilter filter{record_options, nullptr, true};
  auto filtered_topics = filter.filter_topics(topics_services_actions_with_types_);

  EXPECT_THAT(filtered_topics, SizeIs(5));
  for (const auto & topic :
    {"/planning1", "/planning2", "/localization", "/status", "/planning_service/_service_event"})
  {
    EXPECT_TRUE(filtered_topics.find(topic) != filtered_topics.end()) << "Expected topic:" << topic;
  }
}

TEST_F(TestTopicFilter, regex_and_exclude_regex)
{
  rosbag2_transport::RecordOptions record_options;
  record_options.regex = "/invalid.*";
  record_options.exclude_regex = ".invalidated.*";
  rosbag2_transport::TopicFilter filter{record_options, nullptr, true};
  auto filtered_topics = filter.filter_topics(topics_services_actions_with_types_);

  EXPECT_THAT(filtered_topics, SizeIs(7));
  // Matched topic
  EXPECT_TRUE(filtered_topics.find("/invalid_topic") != filtered_topics.end());
  // Matched service
  EXPECT_TRUE(filtered_topics.find("/invalid_service/_service_event") != filtered_topics.end());
  // Matched action
  check_action_interfaces_exist(filtered_topics, "/invalid_action");
}

TEST_F(TestTopicFilter, regex_and_exclude_topics)
{
  rosbag2_transport::RecordOptions record_options;
  record_options.regex = "/invalid.*";
  record_options.exclude_topics = {"/invalidated_topic"};
  rosbag2_transport::TopicFilter filter{record_options, nullptr, true};
  auto filtered_topics = filter.filter_topics(topics_services_actions_with_types_);

  EXPECT_THAT(filtered_topics, SizeIs(13));
  // Matched topic
  EXPECT_TRUE(filtered_topics.find("/invalid_topic") != filtered_topics.end());
  // Matched service
  EXPECT_TRUE(filtered_topics.find("/invalid_service/_service_event") != filtered_topics.end());
  EXPECT_TRUE(filtered_topics.find("/invalidated_service/_service_event") != filtered_topics.end());
  // Matched action
  check_action_interfaces_exist(filtered_topics, "/invalid_action");
  check_action_interfaces_exist(filtered_topics, "/invalidated_action");
}

TEST_F(TestTopicFilter, regex_and_exclude_service_events)
{
  rosbag2_transport::RecordOptions record_options;
  record_options.regex = "/invalid.*";
  record_options.exclude_service_events = {"/invalidated_service/_service_event"};
  rosbag2_transport::TopicFilter filter{record_options, nullptr, true};
  auto filtered_topics = filter.filter_topics(topics_services_actions_with_types_);

  EXPECT_THAT(filtered_topics, SizeIs(13));
  // Matched topic
  EXPECT_TRUE(filtered_topics.find("/invalid_topic") != filtered_topics.end());
  EXPECT_TRUE(filtered_topics.find("/invalidated_topic") != filtered_topics.end());
  // Matched service
  EXPECT_TRUE(filtered_topics.find("/invalid_service/_service_event") != filtered_topics.end());
  // Matched action
  check_action_interfaces_exist(filtered_topics, "/invalid_action");
  check_action_interfaces_exist(filtered_topics, "/invalidated_action");
}

TEST_F(TestTopicFilter, regex_and_exclude_actions)
{
  rosbag2_transport::RecordOptions record_options;
  record_options.regex = "/invalid.*";
  record_options.exclude_actions = {
    "/invalidated_action"
  };
  rosbag2_transport::TopicFilter filter{record_options, nullptr, true};
  auto filtered_topics = filter.filter_topics(topics_services_actions_with_types_);

  EXPECT_THAT(filtered_topics, SizeIs(9));
  // Matched topic
  EXPECT_TRUE(filtered_topics.find("/invalid_topic") != filtered_topics.end());
  EXPECT_TRUE(filtered_topics.find("/invalidated_topic") != filtered_topics.end());
  // Matched service
  EXPECT_TRUE(filtered_topics.find("/invalid_service/_service_event") != filtered_topics.end());
  EXPECT_TRUE(filtered_topics.find("/invalidated_service/_service_event") != filtered_topics.end());
  // Matched action
  check_action_interfaces_exist(filtered_topics, "/invalid_action");
}

TEST_F(TestTopicFilter, regex_filter)
{
  rosbag2_transport::RecordOptions record_options;
  record_options.regex = "^/inval";
  rosbag2_transport::TopicFilter filter{record_options, nullptr, true};
  auto filtered_topics = filter.filter_topics(topics_services_actions_with_types_);

  EXPECT_THAT(filtered_topics, SizeIs(14));

  // Matched topic and service
  for (const auto & topic :
    {"/invalid_topic", "/invalidated_topic", "/invalid_service/_service_event",
      "/invalidated_service/_service_event"})
  {
    EXPECT_TRUE(filtered_topics.find(topic) != filtered_topics.end()) << "Expected topic:" << topic;
  }

  // Matched action
  check_action_interfaces_exist(filtered_topics, "/invalid_action");
  check_action_interfaces_exist(filtered_topics, "/invalidated_action");
}

TEST_F(TestTopicFilter, all_topics_overrides_regex)
{
  rosbag2_transport::RecordOptions record_options;
  record_options.regex = "/status";
  record_options.all_topics = true;
  rosbag2_transport::TopicFilter filter{record_options, nullptr, true};
  auto filtered_topics = filter.filter_topics(topics_services_actions_with_types_);
  EXPECT_THAT(filtered_topics, SizeIs(7));
}

TEST_F(TestTopicFilter, topic_types)
{
  rosbag2_transport::RecordOptions record_options;
  record_options.topic_types = {{"planning_topic_type"}};
  record_options.all_topics = false;
  rosbag2_transport::TopicFilter filter{record_options, nullptr, true};
  auto filtered_topics = filter.filter_topics(topics_services_actions_with_types_);
  EXPECT_THAT(filtered_topics, SizeIs(2));
  EXPECT_TRUE(filtered_topics.find("/planning1") != filtered_topics.end());
  EXPECT_TRUE(filtered_topics.find("/planning2") != filtered_topics.end());
}

TEST_F(TestTopicFilter, topic_types_topic_names_and_regex)
{
  rosbag2_transport::RecordOptions record_options;
  record_options.topic_types = {{"planning_topic_type"}};
  record_options.topics = {{"/localization"}};
  record_options.regex = "^/stat";
  record_options.all_topics = false;
  rosbag2_transport::TopicFilter filter{record_options, nullptr, true};
  auto filtered_topics = filter.filter_topics(topics_services_actions_with_types_);
  EXPECT_THAT(filtered_topics, SizeIs(4));
  EXPECT_TRUE(filtered_topics.find("/planning1") != filtered_topics.end());
  EXPECT_TRUE(filtered_topics.find("/planning2") != filtered_topics.end());
  EXPECT_TRUE(filtered_topics.find("/localization") != filtered_topics.end());
  EXPECT_TRUE(filtered_topics.find("/status") != filtered_topics.end());
}

TEST_F(TestTopicFilter, topic_types_do_not_overlap_with_services)
{
  rosbag2_transport::RecordOptions record_options;
  record_options.topic_types = {{"planning_topic_type"}, {"service/srv/planning_service_Event"}};
  record_options.all_topics = false;
  record_options.all_services = false;
  record_options.services = {"/invalidated_service/_service_event"};
  rosbag2_transport::TopicFilter filter{record_options, nullptr, true};
  auto filtered_topics = filter.filter_topics(topics_services_actions_with_types_);
  EXPECT_THAT(filtered_topics, SizeIs(3));
  EXPECT_TRUE(filtered_topics.find("/planning1") != filtered_topics.end());
  EXPECT_TRUE(filtered_topics.find("/planning2") != filtered_topics.end());
  EXPECT_TRUE(filtered_topics.find(record_options.services[0]) != filtered_topics.end());
}

TEST_F(TestTopicFilter, all_topics_overrides_topic_types)
{
  rosbag2_transport::RecordOptions record_options;
  record_options.topic_types = {{"planning_topic_type"}};
  record_options.all_topics = true;
  rosbag2_transport::TopicFilter filter{record_options, nullptr, true};
  auto filtered_topics = filter.filter_topics(topics_services_actions_with_types_);
  EXPECT_THAT(filtered_topics, SizeIs(7));
}

TEST_F(TestTopicFilter, all_services_overrides_topic_types)
{
  rosbag2_transport::RecordOptions record_options;
  record_options.topic_types = {{"planning_topic_type"}, {"service/srv/planning_service_Event"}};
  record_options.all_topics = false;
  record_options.all_services = true;
  rosbag2_transport::TopicFilter filter{record_options, nullptr, true};
  auto filtered_topics = filter.filter_topics(topics_services_actions_with_types_);
  EXPECT_THAT(filtered_topics, SizeIs(5));
  EXPECT_TRUE(filtered_topics.find("/planning1") != filtered_topics.end());
  EXPECT_TRUE(filtered_topics.find("/planning2") != filtered_topics.end());
  EXPECT_TRUE(filtered_topics.find("/invalidated_service/_service_event") != filtered_topics.end());
}

TEST_F(TestTopicFilter, do_not_print_warning_about_unknown_types_if_topic_is_not_selected) {
  {  // Check for topics explicitly selected via "topics" list
    rosbag2_transport::RecordOptions record_options;
    // Select only one topic with name "/planning1" via topic list
    record_options.topics = {"/planning1"};
    record_options.all_topics = false;
    record_options.all_services = false;
    record_options.all_actions = false;
    rosbag2_transport::TopicFilter filter{record_options, nullptr, false};
    testing::internal::CaptureStderr();
    auto filtered_topics = filter.filter_topics(topics_services_actions_with_types_);
    std::string test_output = testing::internal::GetCapturedStderr();
    ASSERT_EQ(0u, filtered_topics.size());
    EXPECT_TRUE(
      test_output.find(
        "Topic '/invalid_topic' has unknown type 'invalid_topic_type'") == std::string::npos);
    EXPECT_TRUE(
      test_output.find(
        "Topic '/planning1' has unknown type 'planning_topic_type'") != std::string::npos);
  }

  {  // Check for topics selected via regex
    rosbag2_transport::RecordOptions record_options;
    // Select topics wth name starting from "/planning" via regex
    record_options.regex = "^/planning";
    record_options.all_topics = false;
    record_options.all_services = false;
    record_options.all_actions = false;
    rosbag2_transport::TopicFilter filter{record_options, nullptr, false};
    testing::internal::CaptureStderr();
    auto filtered_topics = filter.filter_topics(topics_services_actions_with_types_);
    std::string test_output = testing::internal::GetCapturedStderr();

    ASSERT_EQ(2u, filtered_topics.size());
    EXPECT_TRUE(
      filtered_topics.find("/planning_action/_action/cancel_goal/_service_event") !=
      filtered_topics.end());
    EXPECT_TRUE(
        filtered_topics.find("/planning_action/_action/status") != filtered_topics.end());

    EXPECT_TRUE(
      test_output.find(
        "Topic '/invalid_topic' has unknown type 'invalid_topic_type'") == std::string::npos);
    EXPECT_TRUE(
      test_output.find(
        "Topic '/planning1' has unknown type 'planning_topic_type'") != std::string::npos);
    // Expected to print warning only once for the same topic type
    EXPECT_TRUE(
      test_output.find(
        "Topic '/planning2' has unknown type 'planning_topic_type'") == std::string::npos);

    EXPECT_TRUE(
      test_output.find(
        "Topic '/planning_service/_service_event' has unknown type "
        "'service/srv/planning_service_Event'") != std::string::npos);

    EXPECT_TRUE(test_output.find("'/planning_action/_action/feedback' has unknown type "
      "'unknown_pkg/action/Planning_FeedbackMessage'") != std::string::npos);
    EXPECT_TRUE(test_output.find("'/planning_action/_action/get_result/_service_event' has "
      "unknown type 'unknown_pkg/action/Planning_GetResult_Event'") != std::string::npos);
    EXPECT_TRUE(test_output.find("'/planning_action/_action/send_goal/_service_event'"
      " has unknown type 'unknown_pkg/action/Planning_SendGoal_Event'") != std::string::npos);
  }
}

TEST_F(TestTopicFilter, all_services_overrides_regex)
{
  rosbag2_transport::RecordOptions record_options;
  record_options.regex = "/no_exist_service";
  record_options.all_services = true;

  rosbag2_transport::TopicFilter filter{record_options, nullptr, true};
  auto filtered_topics = filter.filter_topics(topics_services_actions_with_types_);
  EXPECT_THAT(filtered_topics, SizeIs(3));
}

TEST_F(TestTopicFilter, all_topics_and_all_services_overrides_regex)
{
  rosbag2_transport::RecordOptions record_options;
  record_options.regex = "/status";
  record_options.all_topics = true;
  record_options.all_services = true;
  rosbag2_transport::TopicFilter filter{record_options, nullptr, true};
  auto filtered_topics = filter.filter_topics(topics_services_actions_with_types_);
  EXPECT_THAT(filtered_topics, SizeIs(10));
}
