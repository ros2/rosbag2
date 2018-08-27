// Copyright 2018, Bosch Software Innovations GmbH.
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

#include <atomic>
#include <future>
#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rosbag2/rosbag2.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include "rosbag2_integration_test_fixture.hpp"
#include "test_memory_management.hpp"

using namespace ::testing;  // NOLINT
using namespace rosbag2;  // NOLINT

// TODO(Martin-Idel-SI): merge with rosbag2_read_integration_test once signal handling is sorted out
TEST_F(RosBag2IntegrationTestFixture, published_messages_are_recorded)
{
  std::string message_to_publish = "test_message";
  auto serialized_message = serialize_string_message(message_to_publish);

  start_publishing({serialized_message}, "string_topic");
  start_recording({"string_topic"});
  stop_recording();

  auto recorded_messages = get_messages(database_name_);

  ASSERT_THAT(recorded_messages, Not(IsEmpty()));
  EXPECT_THAT(deserialize_string_message(recorded_messages[0]), Eq(message_to_publish));
}
