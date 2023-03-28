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
  auto result = source.get_full_text("rosbag2_test_msgdefs/ComplexMsg");
  ASSERT_EQ(result.encoding, "ros2msg");
  ASSERT_EQ(
    result.encoded_message_definition,
    "rosbag2_test_msgdefs/BasicMsg b\n"
    "\n"
    "================================================================================\n"
    "MSG: rosbag2_test_msgdefs/BasicMsg\n"
    "float32 c\n");
}

TEST(test_local_message_definition_source, can_find_idl_deps)
{
  LocalMessageDefinitionSource source;
  auto result = source.get_full_text("rosbag2_test_msgdefs/msg/ComplexIdl");
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
  auto result = source.get_full_text("rosbag2_test_msgdefs/msg/ComplexMsgDependsOnIdl");
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
