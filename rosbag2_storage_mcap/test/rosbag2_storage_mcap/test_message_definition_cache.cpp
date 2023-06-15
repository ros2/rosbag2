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

#include "gmock/gmock.h"
#include "rosbag2_storage_mcap/message_definition_cache.hpp"

#include <set>
#include <string>

using rosbag2_storage_mcap::internal::Format;
using rosbag2_storage_mcap::internal::MessageDefinitionCache;
using rosbag2_storage_mcap::internal::parse_dependencies;
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
  std::set<std::string> dependencies = parse_dependencies(Format::IDL, sample, "");
  ASSERT_THAT(dependencies, UnorderedElementsAre("rosbag2_storage_mcap_testdata/msg/BasicIdlA",
                                                 "rosbag2_storage_mcap_testdata/msg/BasicIdlB"));
}

TEST(test_message_definition_cache, can_find_msg_deps)
{
  MessageDefinitionCache cache;
  auto [format, content] = cache.get_full_text("rosbag2_storage_mcap_testdata/ComplexMsg");
  ASSERT_EQ(format, Format::MSG);
  ASSERT_EQ(content,
            R"r(rosbag2_storage_mcap_testdata/BasicMsg b

================================================================================
MSG: rosbag2_storage_mcap_testdata/BasicMsg
float32 c
)r");
}

TEST(test_message_definition_cache, can_find_idl_deps)
{
  MessageDefinitionCache cache;
  auto [format, content] = cache.get_full_text("rosbag2_storage_mcap_testdata/msg/ComplexIdl");
  EXPECT_EQ(format, Format::IDL);
  EXPECT_EQ(content,
            R"r(================================================================================
IDL: rosbag2_storage_mcap_testdata/msg/ComplexIdl
#include "rosbag2_storage_mcap_testdata/msg/BasicIdl.idl"

module rosbag2_storage_mcap_testdata {
  module msg {
    struct ComplexIdl {
      rosbag2_storage_mcap_testdata::msg::BasicIdl a;
    };
  };
};

================================================================================
IDL: rosbag2_storage_mcap_testdata/msg/BasicIdl
module rosbag2_storage_mcap_testdata {
  module msg {
    struct BasicIdl {
        float x;
    };
  };
};
)r");
}

TEST(test_message_definition_cache, can_resolve_msg_with_idl_deps)
{
  MessageDefinitionCache cache;
  auto [format, content] =
    cache.get_full_text("rosbag2_storage_mcap_testdata/msg/ComplexMsgDependsOnIdl");
  EXPECT_EQ(format, Format::IDL);
  EXPECT_EQ(content,
            R"r(================================================================================
IDL: rosbag2_storage_mcap_testdata/msg/ComplexMsgDependsOnIdl
// generated from rosidl_adapter/resource/msg.idl.em
// with input from rosbag2_storage_mcap_testdata/msg/ComplexMsgDependsOnIdl.msg
// generated code does not contain a copyright notice

#include "rosbag2_storage_mcap_testdata/msg/BasicIdl.idl"

module rosbag2_storage_mcap_testdata {
  module msg {
    struct ComplexMsgDependsOnIdl {
      rosbag2_storage_mcap_testdata::msg::BasicIdl a;
    };
  };
};

================================================================================
IDL: rosbag2_storage_mcap_testdata/msg/BasicIdl
module rosbag2_storage_mcap_testdata {
  module msg {
    struct BasicIdl {
        float x;
    };
  };
};
)r");
}

TEST(test_local_message_definition_source, no_crash_on_bad_name)
{
  MessageDefinitionCache source;
  std::pair<Format, std::string> result;
  ASSERT_NO_THROW({
    result =
      source.get_full_text("rosbag2_storage_mcap_testdata/action/BasicAction_SetGoal_Request");
  });
  ASSERT_EQ(result.first, rosbag2_storage_mcap::internal::Format::UNKNOWN);
}

TEST(test_message_definition_cache, get_service_message_definitions)
{
  MessageDefinitionCache source;
  auto result = source.get_full_text("rosbag2_storage_mcap_testdata/srv/BasicSrv_Request");
  ASSERT_EQ(result.first, rosbag2_storage_mcap::internal::Format::MSG);
  ASSERT_EQ(result.second, "string req\n");

  result = source.get_full_text("rosbag2_storage_mcap_testdata/srv/BasicSrv_Response");
  ASSERT_EQ(result.first, rosbag2_storage_mcap::internal::Format::MSG);
  ASSERT_EQ(result.second, "\nstring resp\n");
}
