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

#include <cstdlib>
#include <string>
#include <thread>

#include "process_execution_helpers.hpp"

using namespace ::testing;  // NOLINT

class InfoV2EndToEndTestFixture : public Test
{
public:
  InfoV2EndToEndTestFixture()
  {
    database_path_ = _SRC_RESOURCES_DIR_PATH;  // variable defined in CMakeLists.txt
  }

  std::string database_path_;
};

TEST_F(InfoV2EndToEndTestFixture, info_end_to_end_test) {
  internal::CaptureStdout();
  auto exit_code = execute_and_wait_until_completion(
    "ros2 bag info test_bag.bag -s rosbag_v2", database_path_);
  std::string output = internal::GetCapturedStdout();

  EXPECT_THAT(exit_code, Eq(EXIT_SUCCESS));
// The bag size depends on the os and is not asserted, the time is asserted time zone independent
  EXPECT_THAT(output, ContainsRegex(
      "\nFiles:             test_bag\\.bag"
      "\nBag size:          .*B"
      "\nStorage id:        rosbag_v2"
      "\nDuration:          3\\.0s"
      "\nStart:             Dec .+ 2018 .+:.+:06\\.974 \\(1544000766\\.974\\)"
      "\nEnd                Dec .+ 2018 .+:.+:09\\.975 \\(1544000769\\.975\\)"
      "\nMessages:          11"
      "\nTopic information: "));

  EXPECT_THAT(output, HasSubstr(
      "Topic: rosout | Type: rosgraph_msgs/Log | Count: 5 | Serialization Format: rosbag_v2\n"));
  EXPECT_THAT(output, HasSubstr("Topic: string_topic | Type: std_msgs/String | Count: 3 | "
    "Serialization Format: rosbag_v2\n"));
  EXPECT_THAT(output, HasSubstr(
      "Topic: int_topic | Type: std_msgs/Int32 | Count: 3 | Serialization Format: rosbag_v2"
  ));
}


TEST_F(InfoV2EndToEndTestFixture, info_fails_gracefully_if_storage_format_is_not_specified) {
  internal::CaptureStderr();
  auto exit_code =
    execute_and_wait_until_completion("ros2 bag info test_bag.bag", database_path_);
  auto error_output = internal::GetCapturedStderr();

  EXPECT_THAT(exit_code, Eq(EXIT_SUCCESS));
  EXPECT_THAT(error_output, HasSubstr("Could not read metadata for test_bag.bag"));
}
