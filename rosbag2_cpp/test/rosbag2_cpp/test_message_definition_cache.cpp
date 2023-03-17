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
#include "rosbag2_cpp/message_definitions/message_definition_cache.hpp"

using rosbag2_cpp::MessageDefinitionCache;
using rosbag2_cpp::parse_definition_dependencies;
using ::testing::UnorderedElementsAre;

TEST(test_message_definition_cache, can_find_idl_includes)
{
  const char sample[] =
    R"r(
#include "rosbag2_storage_mcap_testdata/msg/BasicIdlA.idl"

#include <rosbag2_storage_mcap_testdata/msg/BasicIdlB.idl>

module rosbag2_storage_mcap_testdata {
  module msg {
    struct ComplexIdl {
      rosbag2_storage_mcap_testdata::msg::BasicIdlA a;
      rosbag2_storage_mcap_testdata::msg::BasicIdlB b;
    };
  };
};

  )r";
  std::set<std::string> dependencies = parse_definition_dependencies(
    MessageDefinitionCache::Format::IDL, sample, "");
  ASSERT_THAT(
    dependencies, UnorderedElementsAre(
      "rosbag2_storage_mcap_testdata/msg/BasicIdlA",
      "rosbag2_storage_mcap_testdata/msg/BasicIdlB"));
}

TEST(test_message_definition_cache, can_find_msg_deps)
{
  MessageDefinitionCache cache;
  auto result = cache.get_full_text("rosbag2_storage_mcap_testdata/ComplexMsg");
  ASSERT_EQ(result.encoding, rosbag2_storage::MessageDefinition::Encoding::ConcatenatedMsg);
  ASSERT_EQ(
    result.encoded_message_definition,
    "rosbag2_storage_mcap_testdata/BasicMsg b\n"
    "\n"
    "================================================================================\n"
    "MSG: rosbag2_storage_mcap_testdata/BasicMsg\n"
    "float32 c\n");
}

TEST(test_message_definition_cache, can_find_idl_deps)
{
  MessageDefinitionCache cache;
  auto result = cache.get_full_text("rosbag2_storage_mcap_testdata/msg/ComplexIdl");
  ASSERT_EQ(result.encoding, rosbag2_storage::MessageDefinition::Encoding::ConcatenatedIdl);
  ASSERT_EQ(
    result.encoded_message_definition,
    "================================================================================\n"
    "IDL: rosbag2_storage_mcap_testdata/msg/ComplexIdl\n"
    "#include \"rosbag2_storage_mcap_testdata/msg/BasicIdl.idl\"\n"
    "\n"
    "module rosbag2_storage_mcap_testdata {\n"
    "  module msg {\n"
    "    struct ComplexIdl {\n"
    "      rosbag2_storage_mcap_testdata::msg::BasicIdl a;\n"
    "    };\n"
    "  };\n"
    "};\n"
    "\n"
    "================================================================================\n"
    "IDL: rosbag2_storage_mcap_testdata/msg/BasicIdl\n"
    "module rosbag2_storage_mcap_testdata {\n"
    "  module msg {\n"
    "    struct BasicIdl {\n"
    "        float x;\n"
    "    };\n"
    "  };\n"
    "};\n");
}

TEST(test_message_definition_cache, can_resolve_msg_with_idl_deps)
{
  MessageDefinitionCache cache;
  auto result = cache.get_full_text("rosbag2_storage_mcap_testdata/msg/ComplexMsgDependsOnIdl");
  ASSERT_EQ(result.encoding, rosbag2_storage::MessageDefinition::Encoding::ConcatenatedIdl);
  ASSERT_EQ(
    result.encoded_message_definition,
    "================================================================================\n"
    "IDL: rosbag2_storage_mcap_testdata/msg/ComplexMsgDependsOnIdl\n"
    "// generated from rosidl_adapter/resource/msg.idl.em\n"
    "// with input from rosbag2_storage_mcap_testdata/msg/ComplexMsgDependsOnIdl.msg\n"
    "// generated code does not contain a copyright notice\n"
    "\n"
    "#include \"rosbag2_storage_mcap_testdata/msg/BasicIdl.idl\"\n"
    "\n"
    "module rosbag2_storage_mcap_testdata {\n"
    "  module msg {\n"
    "    struct ComplexMsgDependsOnIdl {\n"
    "      rosbag2_storage_mcap_testdata::msg::BasicIdl a;\n"
    "    };\n"
    "  };\n"
    "};\n"
    "\n"
    "================================================================================\n"
    "IDL: rosbag2_storage_mcap_testdata/msg/BasicIdl\n"
    "module rosbag2_storage_mcap_testdata {\n"
    "  module msg {\n"
    "    struct BasicIdl {\n"
    "        float x;\n"
    "    };\n"
    "  };\n"
    "};\n");
}
