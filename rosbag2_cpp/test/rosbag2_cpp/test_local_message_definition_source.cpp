// Copyright 2022, Foxglove Technologies. All rights reserved.
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

#include <optional>
#include <ostream>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "gmock/gmock.h"
#include "rosbag2_cpp/message_definitions/local_message_definition_source.hpp"

#include "rosbag2_storage/message_definition.hpp"

using rosbag2_cpp::LocalMessageDefinitionSource;
using rosbag2_cpp::parse_definition_dependencies;
using ::testing::ElementsAre;
using ::testing::HasSubstr;
using ::testing::UnorderedElementsAre;

namespace
{
/// Section header (e.g. "MSG: pkg/msg/Foo", or "" for the top-level ros2msg definition) and body.
using Section = std::pair<std::string, std::string>;

/// Split an encoded message definition into its sections, see docs/message_definition_encoding.md
std::vector<Section> split_sections(const std::string & text)
{
  const std::string delimiter = std::string(80, '=') + "\n";
  std::vector<Section> sections;
  size_t pos = 0;
  bool first = true;
  while (pos <= text.size()) {
    size_t next = text.find(delimiter, pos);
    std::string chunk =
      text.substr(pos, next == std::string::npos ? std::string::npos : next - pos);
    if (next != std::string::npos && !chunk.empty() && chunk.back() == '\n') {
      chunk.pop_back();  // Separator new line before the next delimiter
    }
    if (first) {
      if (!chunk.empty()) {
        sections.emplace_back("", chunk);
      }
      first = false;
    } else {
      const size_t header_end = chunk.find('\n');
      sections.emplace_back(chunk.substr(0, header_end), chunk.substr(header_end + 1));
    }
    if (next == std::string::npos) {
      break;
    }
    pos = next + delimiter.size();
  }
  return sections;
}

std::vector<std::string> section_headers(const std::string & text)
{
  std::vector<std::string> headers;
  for (const auto & section : split_sections(text)) {
    headers.push_back(section.first);
  }
  return headers;
}

std::optional<std::string> section_body(const std::string & text, const std::string & header)
{
  for (const auto & [section_header, body] : split_sections(text)) {
    if (section_header == header) {
      return body;
    }
  }
  return std::nullopt;
}
}  // namespace

TEST(test_local_message_definition_source, can_find_idl_includes)
{
  const char sample[] =
    R"r(
#include "rosbag2_test_msgdefs/msg/BasicIdlA.idl"

#include <rosbag2_test_msgdefs/msg/BasicIdlB.idl>

module rosbag2_test_msgdefs {
  module msg {
    struct ComplexIdl {
      rosbag2_test_msgdefs::msg::BasicIdlA a;
      rosbag2_test_msgdefs::msg::BasicIdlB b;
    };
  };
};

  )r";
  std::set<std::string> dependencies = parse_definition_dependencies(
    LocalMessageDefinitionSource::Format::IDL, sample, "", "");
  EXPECT_THAT(
    dependencies, UnorderedElementsAre(
      "rosbag2_test_msgdefs/msg/BasicIdlA",
      "rosbag2_test_msgdefs/msg/BasicIdlB"));
}

TEST(test_local_message_definition_source, can_find_relative_idl_includes)
{
  const char sample[] =
    R"r(
#include "BasicIdlA.idl"

#include <rosbag2_test_msgdefs/msg/BasicIdlB.idl>

module rosbag2_test_msgdefs {
  module msg {
    struct ComplexIdl {
      rosbag2_test_msgdefs::msg::BasicIdlA a;
      rosbag2_test_msgdefs::msg::BasicIdlB b;
    };
  };
};

  )r";
  std::set<std::string> dependencies = parse_definition_dependencies(
    LocalMessageDefinitionSource::Format::IDL, sample, "rosbag2_test_msgdefs", "msg");
  EXPECT_THAT(
    dependencies, UnorderedElementsAre(
    "rosbag2_test_msgdefs/msg/BasicIdlA",
    "rosbag2_test_msgdefs/msg/BasicIdlB"));
}

TEST(test_local_message_definition_source, can_find_msg_deps)
{
  LocalMessageDefinitionSource source;
  auto result = source.get_full_text_ext("rosbag2_test_msgdefs/ComplexMsg", "/complex_msg_topic");
  EXPECT_EQ(result.encoding, "ros2msg");
  EXPECT_EQ(
    result.encoded_message_definition,
    "rosbag2_test_msgdefs/BasicMsg b\n"
    "\n"
    "================================================================================\n"
    "MSG: rosbag2_test_msgdefs/BasicMsg\n"
    "float32 c\n");
}

TEST(test_local_message_definition_source, can_find_msg_definition_in_nested_subfolder)
{
  LocalMessageDefinitionSource source;
  auto result =
    source.get_full_text_ext("rosbag2_test_msgdefs/msg/nested_sub_dir/AnotherBasicMsg",
                             "/basic_msg_topic");
  EXPECT_EQ(result.encoding, "ros2msg");
  // Note: By design The top-level message definition for MSG format is present first, with no
  // delimiter. All dependent .msg definitions are preceded by a two-line delimiter:
  EXPECT_EQ(result.encoded_message_definition, "float32 c\n");
}

TEST(test_local_message_definition_source, can_find_action_definition_in_nested_subfolder)
{
  LocalMessageDefinitionSource source;
  auto result =
    source.get_full_text_ext("rosbag2_test_msgdefs/nested_sub_dir/action/BasicMsg",
                             "/basic_action_msg/_action/send_goal");
  EXPECT_EQ(result.encoding, "ros2msg");
  EXPECT_EQ(result.encoded_message_definition,
    "================================================================================\n"
    "ACTION: rosbag2_test_msgdefs/nested_sub_dir/action/BasicMsg\n"
    "string goal\n"
    "---\n"
    "string result\n"
    "---\n"
    "string feedback\n"
  );
}

TEST(test_local_message_definition_source, can_find_srv_event_deps_in_msg)
{
  LocalMessageDefinitionSource source;
  auto result = source.get_full_text_ext("rosbag2_test_msgdefs/srv/ComplexSrvMsg_Event",
    "/complex_srv_msg_topic/_service_event");
  EXPECT_EQ(result.encoding, "ros2msg");
  EXPECT_EQ(result.topic_type, "rosbag2_test_msgdefs/srv/ComplexSrvMsg_Event");
  const std::string & text = result.encoded_message_definition;
  EXPECT_THAT(
    section_headers(text), ElementsAre(
      "",
      "MSG: rosbag2_test_msgdefs/srv/ComplexSrvMsg_Request",
      "MSG: rosbag2_test_msgdefs/BasicMsg",
      "MSG: rosbag2_test_msgdefs/srv/ComplexSrvMsg_Response",
      "MSG: service_msgs/msg/ServiceEventInfo",
      "MSG: builtin_interfaces/Time")) << text;
  // The top-level definition describes the service event message published on the topic.
  EXPECT_EQ(
    section_body(text, ""),
    "service_msgs/msg/ServiceEventInfo info\n"
    "rosbag2_test_msgdefs/srv/ComplexSrvMsg_Request[<=1] request\n"
    "rosbag2_test_msgdefs/srv/ComplexSrvMsg_Response[<=1] response\n");
  EXPECT_EQ(
    section_body(text, "MSG: rosbag2_test_msgdefs/srv/ComplexSrvMsg_Request"),
    "rosbag2_test_msgdefs/BasicMsg req\n");
  EXPECT_EQ(
    section_body(text, "MSG: rosbag2_test_msgdefs/srv/ComplexSrvMsg_Response"),
    "rosbag2_test_msgdefs/BasicMsg resp\n");
  EXPECT_EQ(section_body(text, "MSG: rosbag2_test_msgdefs/BasicMsg"), "float32 c\n");
  EXPECT_THAT(
    section_body(text, "MSG: service_msgs/msg/ServiceEventInfo").value_or(""),
    HasSubstr("builtin_interfaces/Time stamp\n"));
}

TEST(test_local_message_definition_source, srv_event_definition_does_not_depend_on_topic_name)
{
  LocalMessageDefinitionSource source;
  const std::string type = "rosbag2_test_msgdefs/srv/ComplexSrvMsg_Event";
  auto with_topic = source.get_full_text_ext(type, "/complex_srv_msg_topic/_service_event");
  auto without_topic = LocalMessageDefinitionSource().get_full_text_ext(type, "");
  EXPECT_EQ(without_topic.encoding, "ros2msg");
  EXPECT_EQ(without_topic.encoded_message_definition, with_topic.encoded_message_definition);
}

TEST(test_local_message_definition_source, can_find_srv_event_deps_in_idl)
{
  LocalMessageDefinitionSource source;
  auto result = source.get_full_text_ext("rosbag2_test_msgdefs/srv/ComplexSrvIdl_Event",
    "/complex_srv_idl_topic/_service_event");
  EXPECT_EQ(result.encoding, "ros2idl");
  EXPECT_EQ(result.topic_type, "rosbag2_test_msgdefs/srv/ComplexSrvIdl_Event");
  const std::string & text = result.encoded_message_definition;
  // All sections must be in IDL format for the definition to be parseable as ros2idl.
  EXPECT_THAT(
    section_headers(text), ElementsAre(
      "IDL: rosbag2_test_msgdefs/srv/ComplexSrvIdl_Event",
      "IDL: rosbag2_test_msgdefs/srv/ComplexSrvIdl",
      "IDL: rosbag2_test_msgdefs/msg/BasicIdl",
      "IDL: service_msgs/msg/ServiceEventInfo",
      "IDL: builtin_interfaces/msg/Time")) << text;
  EXPECT_EQ(
    section_body(text, "IDL: rosbag2_test_msgdefs/srv/ComplexSrvIdl_Event"),
    "#include \"rosbag2_test_msgdefs/srv/ComplexSrvIdl.idl\"\n"
    "#include \"service_msgs/msg/ServiceEventInfo.idl\"\n"
    "\n"
    "module rosbag2_test_msgdefs {\n"
    "  module srv {\n"
    "    struct ComplexSrvIdl_Event {\n"
    "      service_msgs::msg::ServiceEventInfo info;\n"
    "      sequence<rosbag2_test_msgdefs::srv::ComplexSrvIdl_Request, 1> request;\n"
    "      sequence<rosbag2_test_msgdefs::srv::ComplexSrvIdl_Response, 1> response;\n"
    "    };\n"
    "  };\n"
    "};\n");
  // The request and response structures are defined in the IDL file generated from the .srv file
  const std::string srv_idl =
    section_body(text, "IDL: rosbag2_test_msgdefs/srv/ComplexSrvIdl").value_or("");
  EXPECT_THAT(srv_idl, HasSubstr("struct ComplexSrvIdl_Request {"));
  EXPECT_THAT(srv_idl, HasSubstr("struct ComplexSrvIdl_Response {"));
}

TEST(test_local_message_definition_source, can_find_srv_request_and_response_definitions)
{
  LocalMessageDefinitionSource source;
  auto result = source.get_full_text_ext("rosbag2_test_msgdefs/srv/ComplexSrvMsg_Request",
    "/complex_srv_msg_request_topic");
  EXPECT_EQ(result.encoding, "ros2msg");
  EXPECT_EQ(
    result.encoded_message_definition,
    "rosbag2_test_msgdefs/BasicMsg req\n"
    "\n"
    "================================================================================\n"
    "MSG: rosbag2_test_msgdefs/BasicMsg\n"
    "float32 c\n");

  result = source.get_full_text_ext("rosbag2_test_msgdefs/srv/BasicSrv_Response",
    "/basic_srv_response_topic");
  EXPECT_EQ(result.encoding, "ros2msg");
  EXPECT_EQ(result.encoded_message_definition, "string resp\n");
}

TEST(test_local_message_definition_source, can_find_srv_deps_in_msg)
{
  // Plain service type: the content of the .srv file followed by its dependencies
  LocalMessageDefinitionSource source;
  auto result = source.get_full_text_ext("rosbag2_test_msgdefs/srv/ComplexSrvMsg", "");
  EXPECT_EQ(result.encoding, "ros2msg");
  EXPECT_EQ(result.topic_type, "rosbag2_test_msgdefs/srv/ComplexSrvMsg");
  EXPECT_EQ(
    result.encoded_message_definition,
    "================================================================================\n"
    "SRV: rosbag2_test_msgdefs/srv/ComplexSrvMsg\n"
    "rosbag2_test_msgdefs/BasicMsg req\n"
    "---\n"
    "rosbag2_test_msgdefs/BasicMsg resp\n"
    "\n"
    "================================================================================\n"
    "MSG: rosbag2_test_msgdefs/BasicMsg\n"
    "float32 c\n");
}

TEST(test_local_message_definition_source, can_find_srv_deps_in_idl)
{
  // Plain service type: the content of the .srv file followed by its dependencies
  LocalMessageDefinitionSource source;
  auto result = source.get_full_text_ext("rosbag2_test_msgdefs/srv/ComplexSrvIdl", "");
  EXPECT_EQ(result.encoding, "ros2idl");
  EXPECT_EQ(result.topic_type, "rosbag2_test_msgdefs/srv/ComplexSrvIdl");
  EXPECT_EQ(
    result.encoded_message_definition,
    "================================================================================\n"
    "SRV: rosbag2_test_msgdefs/srv/ComplexSrvIdl\n"
    "rosbag2_test_msgdefs/BasicIdl req\n"
    "---\n"
    "rosbag2_test_msgdefs/BasicIdl resp\n"
    "\n"
    "================================================================================\n"
    "IDL: rosbag2_test_msgdefs/BasicIdl\n"
    "module rosbag2_test_msgdefs {\n"
    "  module msg {\n"
    "    struct BasicIdl {\n"
    "        float x;\n"
    "    };\n"
    "  };\n"
    "};\n");
}

TEST(test_local_message_definition_source, can_find_action_deps_in_msg)
{
  const std::string action_type = "rosbag2_test_msgdefs/action/ComplexActionMsg";
  LocalMessageDefinitionSource source;

  // Check action interface send_goal
  {
    auto result = source.get_full_text_ext(
      action_type + "_SendGoal_Event", "/complex_action_msg/_action/send_goal/_service_event");
    EXPECT_EQ(result.encoding, "ros2msg");
    EXPECT_EQ(result.topic_type, action_type + "_SendGoal_Event");
    const std::string & text = result.encoded_message_definition;
    EXPECT_THAT(
      section_headers(text), ElementsAre(
        "",
        "MSG: " + action_type + "_SendGoal_Request",
        "MSG: " + action_type + "_Goal",
        "MSG: rosbag2_test_msgdefs/BasicMsg",
        "MSG: unique_identifier_msgs/UUID",
        "MSG: " + action_type + "_SendGoal_Response",
        "MSG: builtin_interfaces/Time",
        "MSG: service_msgs/msg/ServiceEventInfo")) << text;
    EXPECT_EQ(
      section_body(text, ""),
      "service_msgs/msg/ServiceEventInfo info\n" +
      action_type + "_SendGoal_Request[<=1] request\n" +
      action_type + "_SendGoal_Response[<=1] response\n");
    EXPECT_EQ(
      section_body(text, "MSG: " + action_type + "_SendGoal_Request"),
      "unique_identifier_msgs/UUID goal_id\n" + action_type + "_Goal goal\n");
    EXPECT_EQ(
      section_body(text, "MSG: " + action_type + "_SendGoal_Response"),
      "bool accepted\n"
      "builtin_interfaces/Time stamp\n");
    EXPECT_EQ(
      section_body(text, "MSG: " + action_type + "_Goal"),
      "rosbag2_test_msgdefs/BasicMsg goal\n");
  }

  // Check action interface get_result
  {
    auto result = source.get_full_text_ext(
      action_type + "_GetResult_Event", "/complex_action_msg/_action/get_result/_service_event");
    EXPECT_EQ(result.encoding, "ros2msg");
    const std::string & text = result.encoded_message_definition;
    EXPECT_THAT(
      section_headers(text), ElementsAre(
        "",
        "MSG: " + action_type + "_GetResult_Request",
        "MSG: unique_identifier_msgs/UUID",
        "MSG: " + action_type + "_GetResult_Response",
        "MSG: " + action_type + "_Result",
        "MSG: rosbag2_test_msgdefs/BasicMsg",
        "MSG: service_msgs/msg/ServiceEventInfo",
        "MSG: builtin_interfaces/Time")) << text;
    EXPECT_EQ(
      section_body(text, ""),
      "service_msgs/msg/ServiceEventInfo info\n" +
      action_type + "_GetResult_Request[<=1] request\n" +
      action_type + "_GetResult_Response[<=1] response\n");
    EXPECT_EQ(
      section_body(text, "MSG: " + action_type + "_GetResult_Request"),
      "unique_identifier_msgs/UUID goal_id\n");
    EXPECT_EQ(
      section_body(text, "MSG: " + action_type + "_GetResult_Response"),
      "int8 status\n" + action_type + "_Result result\n");
    EXPECT_EQ(
      section_body(text, "MSG: " + action_type + "_Result"),
      "rosbag2_test_msgdefs/BasicMsg result\n");
  }

  // Check action interface feedback
  {
    auto result = source.get_full_text_ext(
      action_type + "_FeedbackMessage", "/complex_action_msg/_action/feedback");
    EXPECT_EQ(result.encoding, "ros2msg");
    EXPECT_EQ(
      result.encoded_message_definition,
      "unique_identifier_msgs/UUID goal_id\n" +
      action_type + "_Feedback feedback\n"
      "\n"
      "================================================================================\n"
      "MSG: " + action_type + "_Feedback\n"
      "rosbag2_test_msgdefs/BasicMsg feedback\n"
      "\n"
      "================================================================================\n"
      "MSG: rosbag2_test_msgdefs/BasicMsg\n"
      "float32 c\n"
      "\n"
      "================================================================================\n"
      "MSG: unique_identifier_msgs/UUID\n" +
      section_body(result.encoded_message_definition, "MSG: unique_identifier_msgs/UUID")
      .value_or("<missing>"));
  }
}

TEST(test_local_message_definition_source, can_find_action_cancel_goal_and_status_deps)
{
  // The cancel_goal and status action interfaces use types from action_msgs, which are the same
  // for all actions. They don't depend on prior lookups of other interfaces of the same action.
  for (const std::string action_name : {"/complex_action_msg", "/complex_action_idl"}) {
    LocalMessageDefinitionSource source;
    auto result = source.get_full_text_ext(
      "action_msgs/srv/CancelGoal_Event", action_name + "/_action/cancel_goal/_service_event");
    EXPECT_EQ(result.encoding, "ros2msg");
    EXPECT_EQ(result.topic_type, "action_msgs/srv/CancelGoal_Event");
    const std::string & text = result.encoded_message_definition;
    EXPECT_THAT(
      section_headers(text), ElementsAre(
        "",
        "MSG: action_msgs/srv/CancelGoal_Request",
        "MSG: action_msgs/GoalInfo",
        "MSG: builtin_interfaces/Time",
        "MSG: unique_identifier_msgs/UUID",
        "MSG: action_msgs/srv/CancelGoal_Response",
        "MSG: service_msgs/msg/ServiceEventInfo")) << text;
    EXPECT_EQ(
      section_body(text, ""),
      "service_msgs/msg/ServiceEventInfo info\n"
      "action_msgs/srv/CancelGoal_Request[<=1] request\n"
      "action_msgs/srv/CancelGoal_Response[<=1] response\n");
    EXPECT_THAT(
      section_body(text, "MSG: action_msgs/srv/CancelGoal_Request").value_or(""),
      HasSubstr("GoalInfo goal_info\n"));
    EXPECT_THAT(
      section_body(text, "MSG: action_msgs/srv/CancelGoal_Response").value_or(""),
      HasSubstr("GoalInfo[] goals_canceling\n"));

    result = source.get_full_text_ext(
      "action_msgs/msg/GoalStatusArray", action_name + "/_action/status");
    EXPECT_EQ(result.encoding, "ros2msg");
    EXPECT_EQ(result.topic_type, "action_msgs/msg/GoalStatusArray");
    EXPECT_THAT(
      section_headers(result.encoded_message_definition), ElementsAre(
        "",
        "MSG: action_msgs/GoalStatus",
        "MSG: action_msgs/GoalInfo",
        "MSG: builtin_interfaces/Time",
        "MSG: unique_identifier_msgs/UUID")) << result.encoded_message_definition;
    EXPECT_THAT(result.encoded_message_definition, HasSubstr("GoalStatus[] status_list"));
  }
}

TEST(test_local_message_definition_source, can_find_action_deps_in_idl)
{
  const std::string action_type = "rosbag2_test_msgdefs/action/ComplexActionIdl";
  LocalMessageDefinitionSource source;

  // Check action interface send_goal
  {
    auto result = source.get_full_text_ext(
      action_type + "_SendGoal_Event", "/complex_action_idl/_action/send_goal/_service_event");
    EXPECT_EQ(result.encoding, "ros2idl");
    EXPECT_EQ(result.topic_type, action_type + "_SendGoal_Event");
    const std::string & text = result.encoded_message_definition;
    EXPECT_THAT(
      section_headers(text), ElementsAre(
        "IDL: " + action_type + "_SendGoal_Event",
        "IDL: " + action_type + "_SendGoal",
        "IDL: builtin_interfaces/msg/Time",
        "IDL: " + action_type,
        "IDL: rosbag2_test_msgdefs/msg/BasicIdl",
        "IDL: unique_identifier_msgs/msg/UUID",
        "IDL: service_msgs/msg/ServiceEventInfo")) << text;
    EXPECT_EQ(
      section_body(text, "IDL: " + action_type + "_SendGoal_Event"),
      "#include \"rosbag2_test_msgdefs/action/ComplexActionIdl_SendGoal.idl\"\n"
      "#include \"service_msgs/msg/ServiceEventInfo.idl\"\n"
      "\n"
      "module rosbag2_test_msgdefs {\n"
      "  module action {\n"
      "    struct ComplexActionIdl_SendGoal_Event {\n"
      "      service_msgs::msg::ServiceEventInfo info;\n"
      "      sequence<rosbag2_test_msgdefs::action::ComplexActionIdl_SendGoal_Request, 1> "
      "request;\n"
      "      sequence<rosbag2_test_msgdefs::action::ComplexActionIdl_SendGoal_Response, 1> "
      "response;\n"
      "    };\n"
      "  };\n"
      "};\n");
    EXPECT_EQ(
      section_body(text, "IDL: " + action_type + "_SendGoal"),
      "#include \"builtin_interfaces/msg/Time.idl\"\n"
      "#include \"rosbag2_test_msgdefs/action/ComplexActionIdl.idl\"\n"
      "#include \"unique_identifier_msgs/msg/UUID.idl\"\n"
      "\n"
      "module rosbag2_test_msgdefs {\n"
      "  module action {\n"
      "    struct ComplexActionIdl_SendGoal_Request {\n"
      "      unique_identifier_msgs::msg::UUID goal_id;\n"
      "      rosbag2_test_msgdefs::action::ComplexActionIdl_Goal goal;\n"
      "    };\n"
      "    struct ComplexActionIdl_SendGoal_Response {\n"
      "      boolean accepted;\n"
      "      builtin_interfaces::msg::Time stamp;\n"
      "    };\n"
      "  };\n"
      "};\n");
    // The goal, result and feedback structures are defined in the IDL file generated from the
    // .action file
    EXPECT_THAT(
      section_body(text, "IDL: " + action_type).value_or(""),
      HasSubstr("struct ComplexActionIdl_Goal {"));
  }

  // Check action interface get_result
  {
    auto result = source.get_full_text_ext(
      action_type + "_GetResult_Event", "/complex_action_idl/_action/get_result/_service_event");
    EXPECT_EQ(result.encoding, "ros2idl");
    const std::string & text = result.encoded_message_definition;
    EXPECT_THAT(
      section_headers(text), ElementsAre(
        "IDL: " + action_type + "_GetResult_Event",
        "IDL: " + action_type + "_GetResult",
        "IDL: " + action_type,
        "IDL: rosbag2_test_msgdefs/msg/BasicIdl",
        "IDL: unique_identifier_msgs/msg/UUID",
        "IDL: service_msgs/msg/ServiceEventInfo",
        "IDL: builtin_interfaces/msg/Time")) << text;
    EXPECT_EQ(
      section_body(text, "IDL: " + action_type + "_GetResult"),
      "#include \"rosbag2_test_msgdefs/action/ComplexActionIdl.idl\"\n"
      "#include \"unique_identifier_msgs/msg/UUID.idl\"\n"
      "\n"
      "module rosbag2_test_msgdefs {\n"
      "  module action {\n"
      "    struct ComplexActionIdl_GetResult_Request {\n"
      "      unique_identifier_msgs::msg::UUID goal_id;\n"
      "    };\n"
      "    struct ComplexActionIdl_GetResult_Response {\n"
      "      int8 status;\n"
      "      rosbag2_test_msgdefs::action::ComplexActionIdl_Result result;\n"
      "    };\n"
      "  };\n"
      "};\n");
  }

  // Check action interface feedback
  {
    auto result = source.get_full_text_ext(
      action_type + "_FeedbackMessage", "/complex_action_idl/_action/feedback");
    EXPECT_EQ(result.encoding, "ros2idl");
    const std::string & text = result.encoded_message_definition;
    EXPECT_THAT(
      section_headers(text), ElementsAre(
        "IDL: " + action_type + "_FeedbackMessage",
        "IDL: " + action_type,
        "IDL: rosbag2_test_msgdefs/msg/BasicIdl",
        "IDL: unique_identifier_msgs/msg/UUID")) << text;
    EXPECT_EQ(
      section_body(text, "IDL: " + action_type + "_FeedbackMessage"),
      "#include \"rosbag2_test_msgdefs/action/ComplexActionIdl.idl\"\n"
      "#include \"unique_identifier_msgs/msg/UUID.idl\"\n"
      "\n"
      "module rosbag2_test_msgdefs {\n"
      "  module action {\n"
      "    struct ComplexActionIdl_FeedbackMessage {\n"
      "      unique_identifier_msgs::msg::UUID goal_id;\n"
      "      rosbag2_test_msgdefs::action::ComplexActionIdl_Feedback feedback;\n"
      "    };\n"
      "  };\n"
      "};\n");
  }
}

TEST(test_local_message_definition_source, can_find_action_definition)
{
  // Plain action type: the content of the .action file followed by its dependencies
  LocalMessageDefinitionSource source;
  auto result = source.get_full_text_ext("rosbag2_test_msgdefs/action/ComplexActionMsg", "");
  EXPECT_EQ(result.encoding, "ros2msg");
  EXPECT_EQ(
    result.encoded_message_definition,
    "================================================================================\n"
    "ACTION: rosbag2_test_msgdefs/action/ComplexActionMsg\n"
    "rosbag2_test_msgdefs/BasicMsg goal\n"
    "---\n"
    "rosbag2_test_msgdefs/BasicMsg result\n"
    "---\n"
    "rosbag2_test_msgdefs/BasicMsg feedback\n"
    "\n"
    "================================================================================\n"
    "MSG: rosbag2_test_msgdefs/BasicMsg\n"
    "float32 c\n");
}

TEST(test_local_message_definition_source, gracefully_handle_unknown_srv_and_action)
{
  LocalMessageDefinitionSource source;
  for (const std::string type : {
    "rosbag2_test_msgdefs/srv/UnknownSrv_Event",
    "rosbag2_test_msgdefs/srv/UnknownSrv",
    "rosbag2_test_msgdefs/action/UnknownAction_SendGoal_Event",
    "rosbag2_test_msgdefs/action/UnknownAction_FeedbackMessage",
    "rosbag2_test_msgdefs/action/UnknownAction",
    "not_found_msgdefs_pkg/srv/UnknownSrv_Event"})
  {
    rosbag2_storage::MessageDefinition result;
    EXPECT_NO_THROW({result = source.get_full_text_ext(type, "/topic");}) << type;
    EXPECT_EQ(result.encoding, "unknown") << type;
    EXPECT_EQ(result.encoded_message_definition, "") << type;
  }
}

TEST(test_local_message_definition_source, can_find_idl_deps)
{
  LocalMessageDefinitionSource source;
  auto result = source.get_full_text_ext(
    "rosbag2_test_msgdefs/msg/ComplexIdl", "/complex_idl_topic");
  EXPECT_EQ(result.encoding, "ros2idl");
  EXPECT_EQ(
    result.encoded_message_definition,
    "================================================================================\n"
    "IDL: rosbag2_test_msgdefs/msg/ComplexIdl\n"
    "#include \"rosbag2_test_msgdefs/msg/BasicIdl.idl\"\n"
    "\n"
    "module rosbag2_test_msgdefs {\n"
    "  module msg {\n"
    "    struct ComplexIdl {\n"
    "      rosbag2_test_msgdefs::msg::BasicIdl a;\n"
    "    };\n"
    "  };\n"
    "};\n"
    "\n"
    "================================================================================\n"
    "IDL: rosbag2_test_msgdefs/msg/BasicIdl\n"
    "module rosbag2_test_msgdefs {\n"
    "  module msg {\n"
    "    struct BasicIdl {\n"
    "        float x;\n"
    "    };\n"
    "  };\n"
    "};\n");
}

TEST(test_local_message_definition_source, can_find_relative_idl_deps)
{
  LocalMessageDefinitionSource source;
  auto result = source.get_full_text_ext(
    "rosbag2_test_msgdefs/msg/ComplexWithRelativeDep", "/complex_idl_topic");
  EXPECT_EQ(result.encoding, "ros2idl");
  EXPECT_EQ(
    result.encoded_message_definition,
    "================================================================================\n"
    "IDL: rosbag2_test_msgdefs/msg/ComplexWithRelativeDep\n"
    "#include \"BasicIdl.idl\"\n"
    "\n"
    "module rosbag2_test_msgdefs {\n"
    "  module msg {\n"
    "    struct ComplexWithRelativeDep {\n"
    "      rosbag2_test_msgdefs::msg::BasicIdl a;\n"
    "    };\n"
    "  };\n"
    "};\n"
    "\n"
    "================================================================================\n"
    "IDL: rosbag2_test_msgdefs/msg/BasicIdl\n"
    "module rosbag2_test_msgdefs {\n"
    "  module msg {\n"
    "    struct BasicIdl {\n"
    "        float x;\n"
    "    };\n"
    "  };\n"
    "};\n");
}

TEST(test_local_message_definition_source, can_resolve_msg_with_idl_deps)
{
  LocalMessageDefinitionSource source;
  auto result = source.get_full_text_ext(
    "rosbag2_test_msgdefs/msg/ComplexMsgDependsOnIdl", "/complex_msg_depends_on_idl_topic");
  EXPECT_EQ(result.encoding, "ros2idl");
  EXPECT_EQ(
    result.encoded_message_definition,
    "================================================================================\n"
    "IDL: rosbag2_test_msgdefs/msg/ComplexMsgDependsOnIdl\n"
    "// generated from rosidl_adapter/resource/msg.idl.em\n"
    "// with input from rosbag2_test_msgdefs/msg/ComplexMsgDependsOnIdl.msg\n"
    "// generated code does not contain a copyright notice\n"
    "\n"
    "#include \"rosbag2_test_msgdefs/msg/BasicIdl.idl\"\n"
    "\n"
    "module rosbag2_test_msgdefs {\n"
    "  module msg {\n"
    "    struct ComplexMsgDependsOnIdl {\n"
    "      rosbag2_test_msgdefs::msg::BasicIdl a;\n"
    "    };\n"
    "  };\n"
    "};\n"
    "\n"
    "================================================================================\n"
    "IDL: rosbag2_test_msgdefs/msg/BasicIdl\n"
    "module rosbag2_test_msgdefs {\n"
    "  module msg {\n"
    "    struct BasicIdl {\n"
    "        float x;\n"
    "    };\n"
    "  };\n"
    "};\n");
}

TEST(test_local_message_definition_source, no_crash_on_bad_name)
{
  LocalMessageDefinitionSource source;
  rosbag2_storage::MessageDefinition result;

  // The following type names are not valid, but it should not crash
  EXPECT_NO_THROW(
  {
    // The typename without preceding package name
    result = source.get_full_text_ext("/msg/String", "/msg_topic");
  });
  EXPECT_EQ(result.encoding, "unknown");

  EXPECT_NO_THROW(
  {
    // Missing the actual type name after format specifier
    result = source.get_full_text_ext("std_msgs/msg/", "/msg_topic");
  });
  EXPECT_EQ(result.encoding, "unknown");

  EXPECT_NO_THROW(
  {
    // Missing package name before the first slash
    result = source.get_full_text_ext("/String", "/msg_topic");
  });
  EXPECT_EQ(result.encoding, "unknown");

  EXPECT_NO_THROW(
  {
    // Hyphens are not allowed in package names
    result = source.get_full_text_ext("std-msgs/String", "/msg_topic");
  });
  EXPECT_EQ(result.encoding, "unknown");

  EXPECT_NO_THROW(
  {
    // File extensions are not allowed
    result = source.get_full_text_ext("std_msgs/String.msg", "/msg_topic");
  });
  EXPECT_EQ(result.encoding, "unknown");
}

TEST(test_local_message_definition_source, gracefully_handle_unknown_msg)
{
  LocalMessageDefinitionSource source;
  rosbag2_storage::MessageDefinition result;
  EXPECT_NO_THROW(
  {
    result = source.get_full_text_ext("rosbag2_test_msgdefs/msg/UnknownMessage",
      "/unknown_msg_topic");
  });
  EXPECT_EQ(result.encoding, "unknown");

  // No throw for not found message definition package name
  EXPECT_NO_THROW(
  {
    result = source.get_full_text_ext(
      "not_found_msgdefs_pkg/msg/UnknownMessage", "/not_found_msgdefs_pkg_topic");
  });
  EXPECT_EQ(result.encoding, "unknown");
}
