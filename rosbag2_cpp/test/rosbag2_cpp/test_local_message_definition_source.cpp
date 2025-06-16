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

#include <set>
#include <string>

#include "gmock/gmock.h"
#include "rosbag2_cpp/message_definitions/local_message_definition_source.hpp"

using rosbag2_cpp::LocalMessageDefinitionSource;
using rosbag2_cpp::parse_definition_dependencies;
using ::testing::UnorderedElementsAre;

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
    LocalMessageDefinitionSource::Format::IDL, sample, "");
  ASSERT_THAT(
    dependencies, UnorderedElementsAre(
      "rosbag2_test_msgdefs/msg/BasicIdlA",
      "rosbag2_test_msgdefs/msg/BasicIdlB"));
}

TEST(test_local_message_definition_source, can_find_msg_deps)
{
  LocalMessageDefinitionSource source;
  auto result = source.get_full_text_ext("rosbag2_test_msgdefs/ComplexMsg", "/complex_msg_topic");
  ASSERT_EQ(result.encoding, "ros2msg");
  ASSERT_EQ(
    result.encoded_message_definition,
    "rosbag2_test_msgdefs/BasicMsg b\n"
    "\n"
    "================================================================================\n"
    "MSG: rosbag2_test_msgdefs/BasicMsg\n"
    "float32 c\n");
}

TEST(test_local_message_definition_source, can_find_srv_deps_in_msg)
{
  LocalMessageDefinitionSource source;
  auto result = source.get_full_text_ext("rosbag2_test_msgdefs/srv/ComplexSrvMsg_Event",
    "/complex_srv_msg_topic/_service_event");
  ASSERT_EQ(result.encoding, "ros2msg");
  ASSERT_EQ(
    result.encoded_message_definition,
    "================================================================================\n"
    "SRV: rosbag2_test_msgdefs/srv/ComplexSrvMsg\n"
    "rosbag2_test_msgdefs/BasicMsg req\n"
    "---\n"
    "rosbag2_test_msgdefs/BasicMsg resp\n"
    "\n"
    "================================================================================\n"
    "MSG: rosbag2_test_msgdefs/BasicMsg\n"
    "float32 c\n") << result.encoded_message_definition << std::endl;
}

TEST(test_local_message_definition_source, can_find_srv_deps_in_idl)
{
  LocalMessageDefinitionSource source;
  auto result = source.get_full_text_ext("rosbag2_test_msgdefs/srv/ComplexSrvIdl_Event",
    "/complex_srv_idl_topic/_service_event");
  ASSERT_EQ(result.encoding, "ros2idl");
  ASSERT_EQ(
    result.encoded_message_definition,
    "================================================================================\n"
    "SRV: rosbag2_test_msgdefs/srv/ComplexSrvIdl\n"
    "rosbag2_test_msgdefs/BasicIdl req\n"
    "---\n"
    "rosbag2_test_msgdefs/BasicIdl resp\n"
    "\n"
    "================================================================================\n"
    "MSG: rosbag2_test_msgdefs/BasicIdl\n"
    "\n"
    "================================================================================\n"
    "IDL: rosbag2_test_msgdefs/BasicIdl\n"
    "module rosbag2_test_msgdefs {\n"
    "  module msg {\n"
    "    struct BasicIdl {\n"
    "        float x;\n"
    "    };\n"
    "  };\n"
    "};\n") << result.encoded_message_definition << std::endl;
}

TEST(test_local_message_definition_source, can_find_action_deps_in_msg)
{
  auto check_result = [](const rosbag2_storage::MessageDefinition & result) {
      ASSERT_EQ(result.encoding, "ros2msg");
      ASSERT_EQ(
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
        "float32 c\n") << result.encoded_message_definition << std::endl;
    };

  LocalMessageDefinitionSource source;

  // Check action interface send_goal
  {
    auto result = source.get_full_text_ext(
      "rosbag2_test_msgdefs/action/ComplexActionMsg_SendGoal_Event",
      "/complex_action_msg/_action/send_goal/_service_event");
    check_result(result);
  }

  // Check action interface get_result
  {
    auto result = source.get_full_text_ext(
      "rosbag2_test_msgdefs/action/ComplexActionMsg_GetResult_Event",
      "/complex_action_msg/_action/get_result/_service_event");
    check_result(result);
  }

  // Check action interface feedback
  {
    auto result = source.get_full_text_ext(
      "rosbag2_test_msgdefs/action/ComplexActionMsg_FeedbackMessage",
      "/complex_action_msg/_action/feedback");
    check_result(result);
  }

  // Known limitation
  // get_full_text_ext() cannot return action definition for action interface 'cancel_goal' and
  // 'status'.
  // This issue will be resolved in future versions.
}

TEST(test_local_message_definition_source, can_find_action_deps_in_idl)
{
  auto check_result = [](const rosbag2_storage::MessageDefinition & result) {
      ASSERT_EQ(result.encoding, "ros2idl");
      ASSERT_EQ(
        result.encoded_message_definition,
        "================================================================================\n"
        "ACTION: rosbag2_test_msgdefs/action/ComplexActionIdl\n"
        "rosbag2_test_msgdefs/BasicIdl goal\n"
        "---\n"
        "rosbag2_test_msgdefs/BasicIdl result\n"
        "---\n"
        "rosbag2_test_msgdefs/BasicIdl feedback\n"
        "\n"
        "================================================================================\n"
        "MSG: rosbag2_test_msgdefs/BasicIdl\n"
        "\n"
        "================================================================================\n"
        "IDL: rosbag2_test_msgdefs/BasicIdl\n"
        "module rosbag2_test_msgdefs {\n"
        "  module msg {\n"
        "    struct BasicIdl {\n"
        "        float x;\n"
        "    };\n"
        "  };\n"
        "};\n") << result.encoded_message_definition << std::endl;
    };

  LocalMessageDefinitionSource source;

  // Check action interface send_goal
  {
    auto result = source.get_full_text_ext(
      "rosbag2_test_msgdefs/action/ComplexActionIdl_SendGoal_Event",
      "/complex_action_idl/_action/send_goal/_service_event");
    check_result(result);
  }

  // Check action interface get_result
  {
    auto result = source.get_full_text_ext(
      "rosbag2_test_msgdefs/action/ComplexActionIdl_GetResult_Event",
      "/complex_action_idl/_action/get_result/_service_event");
    check_result(result);
  }

  // Check action interface feedback
  {
    auto result = source.get_full_text_ext(
      "rosbag2_test_msgdefs/action/ComplexActionIdl_FeedbackMessage",
      "/complex_action_idl/_action/feedback");
    check_result(result);
  }

  // Known limitation
  // get_full_text_ext() cannot return action definition for action interface 'cancel_goal' and
  // 'status'.
  // This issue will be resolved in future versions.
}

TEST(test_local_message_definition_source, can_find_idl_deps)
{
  LocalMessageDefinitionSource source;
  auto result = source.get_full_text_ext(
    "rosbag2_test_msgdefs/msg/ComplexIdl", "/complex_idl_topic");
  ASSERT_EQ(result.encoding, "ros2idl");
  ASSERT_EQ(
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

TEST(test_local_message_definition_source, can_resolve_msg_with_idl_deps)
{
  LocalMessageDefinitionSource source;
  auto result = source.get_full_text_ext(
    "rosbag2_test_msgdefs/msg/ComplexMsgDependsOnIdl", "/complex_msg_depends_on_idl_topic");
  ASSERT_EQ(result.encoding, "ros2idl");
  ASSERT_EQ(
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
  ASSERT_NO_THROW(
  {
    result = source.get_full_text_ext("rosbag2_test_msgdefs/idl/BasicSrv_Request", "/basic_srv");
  });
  ASSERT_EQ(result.encoding, "unknown");
}

TEST(test_local_message_definition_source, throw_definition_not_found_for_unknown_msg)
{
  LocalMessageDefinitionSource source;
  ASSERT_THROW(
  {
    source.get_full_text_ext("rosbag2_test_msgdefs/msg/UnknownMessage", "/unknown_msg_topic");
  }, rosbag2_cpp::DefinitionNotFoundError);

  // Throw DefinitionNotFoundError for not found message definition package name
  ASSERT_THROW(
  {
    source.get_full_text_ext(
      "not_found_msgdefs_pkg/msg/UnknownMessage", "/not_found_msgdefs_pkg_topic");
  }, rosbag2_cpp::DefinitionNotFoundError);
}
