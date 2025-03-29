// Copyright 2025 Sony Group Corporation.
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
#include  <gmock/gmock.h>

#include <string>
#include <tuple>
#include <vector>

#include "rosbag2_cpp/action_utils.hpp"
#include "rosbag2_test_common/memory_management.hpp"

using namespace ::testing;  // NOLINT
using namespace rosbag2_test_common; // NOLINT

class ActionUtilsTest : public Test
{
public:
  MemoryManagement memory_management_;
};

TEST_F(ActionUtilsTest, check_is_topic_related_to_action)
{
  std::vector<std::pair<std::tuple<std::string, std::string>, bool>> all_test_data =
  {
    {{"/abc/_action/feedback", "package/action/xyz_FeedbackMessage"}, true},
    {{"/abc/action/feedback", "package/action/xyz_FeedbackMessage"}, false},
    {{"/abc/_action/feedback", "package/xyz_FeedbackMessage"}, false},
    {{"/abc/_action/feedback", "package/action/xyz"}, false},
    {{"/abc/_action/get_result/_service_event", "package/action/xyz_GetResult_Event"}, true},
    {{"/abc/action/get_result/_service_event", "package/action/xyz_GetResult_Event"}, false},
    {{"/abc/_action/get_result", "package/action/xyz_GetResult_Event"}, false},
    {{"/abc/_action/get_result/service_event", "package/action/xyz_GetResult_Event"}, false},
    {{"/abc/_action/get_result/_service_event", "package/xyz_GetResult_Event"}, false},
    {{"/abc/_action/get_result/_service_event", "package/action/xyz_GetResult"}, false},
    {{"/abc/_action/send_goal/_service_event", "package/action/xyz_SendGoal_Event"}, true},
    {{"/abc/action/send_goal/_service_event", "package/action/xyz_SendGoal_Event"}, false},
    {{"/abc/_action/send_goal", "package/action/xyz_SendGoal_Event"}, false},
    {{"/abc/_action/send_goal/service_event", "package/action/xyz_GetResult_Event"}, false},
    {{"/abc/_action/send_goal/_service_event", "package/xyz_SendGoal_Event"}, false},
    {{"/abc/_action/send_goal/_service_event", "package/action/xyz_SendGoal"}, false},
    {{"/abc/_action/cancel_goal/_service_event", "action_msgs/srv/CancelGoal_Event"}, true},
    {{"/abc/action/cancel_goal/_service_event", "action_msgs/srv/CancelGoal_Event"}, false},
    {{"/abc/_action/cancel_goal", "action_msgs/srv/CancelGoal_Event"}, false},
    {{"/abc/_action/cancel_goal/_service_event", "action_msgs/CancelGoal_Event"}, false},
    {{"/abc/_action/cancel_goal/service_event", "action_msgs/srv/CancelGoal"}, false},
    {{"/abc/_action/status", "action_msgs/msg/GoalStatusArray"}, true},
    {{"/abc/action/status", "action_msgs/msg/GoalStatusArray"}, false},
    {{"/abc/_action/status", "action_msgs/GoalStatusArray"}, false},
    {{"/abc/_action/status", "action_msgs/msg/GoalStatus"}, false}
  };

  for (const auto & [topic_name_and_type, expected_result] : all_test_data) {
    const auto & [topic_name, topic_type] = topic_name_and_type;
    EXPECT_TRUE(rosbag2_cpp::is_topic_belong_to_action(topic_name, topic_type) == expected_result);
  }
}

TEST_F(ActionUtilsTest, check_action_topic_name_to_action_name)
{
  std::vector<std::pair<std::string, std::string>> all_test_data =
  {
    {"/abc/_action/feedback", "/abc"},
    {"/abc/_action/get_result/_service_event", "/abc"},
    {"/abc/_action/send_goal/_service_event", "/abc"},
    {"/abc/_action/cancel_goal/_service_event", "/abc"},
    {"/abc/_action/status", "/abc"},
    {"/_action/status", ""},
    {"/abc/action/status", ""}
  };

  for (const auto & [topic_name, action_name] : all_test_data) {
    EXPECT_TRUE(rosbag2_cpp::action_interface_name_to_action_name(topic_name) == action_name);
  }
}

TEST_F(ActionUtilsTest, check_action_topic_type_to_action_type)
{
  std::vector<std::pair<std::string, std::string>> all_test_data =
  {
    {"package/action/xyz_FeedbackMessage", "package/action/xyz"},
    {"package/action/xyz_GetResult_Event", "package/action/xyz"},
    {"package/action/xyz_SendGoal_Event", "package/action/xyz"},
    {"action_msgs/srv/CancelGoal_Event", ""},
    {"action_msgs/msg/GoalStatusArray", ""}
  };

  for (const auto & [topic_type, action_type] : all_test_data) {
    EXPECT_EQ(rosbag2_cpp::get_action_type_for_info(topic_type), action_type);
  }
}

TEST_F(ActionUtilsTest, check_get_action_interface_type)
{
  std::vector<std::pair<std::string, rosbag2_cpp::ActionInterfaceType>> all_test_data =
  {
    {"/abc/_action/feedback", rosbag2_cpp::ActionInterfaceType::Feedback},
    {"/abc/_action/get_result/_service_event", rosbag2_cpp::ActionInterfaceType::GetResultEvent},
    {"/abc/_action/send_goal/_service_event", rosbag2_cpp::ActionInterfaceType::SendGoalEvent},
    {"/abc/_action/cancel_goal/_service_event", rosbag2_cpp::ActionInterfaceType::CancelGoalEvent},
    {"/abc/_action/status", rosbag2_cpp::ActionInterfaceType::Status},
    {"/_action/status", rosbag2_cpp::ActionInterfaceType::Unknown},
    {"/abc/action/status", rosbag2_cpp::ActionInterfaceType::Unknown}
  };

  for (const auto & [topic_name, action_interface_type] : all_test_data) {
    EXPECT_EQ(rosbag2_cpp::get_action_interface_type(topic_name), action_interface_type);
  }
}

TEST_F(ActionUtilsTest, check_action_name_to_action_topic_name)
{
  std::string action_name = "abc";
  std::vector<std::string> expected_action_topics =
  {
    "abc/_action/feedback",
    "abc/_action/get_result/_service_event",
    "abc/_action/send_goal/_service_event",
    "abc/_action/cancel_goal/_service_event",
    "abc/_action/status"
  };

  auto output_action_topics = rosbag2_cpp::action_name_to_action_interface_names(action_name);

  for (auto & topic : output_action_topics) {
    EXPECT_TRUE(std::find(
      expected_action_topics.begin(),
      expected_action_topics.end(),
      topic) != expected_action_topics.end());
  }
}
