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

#include "rosbag2_test_common/process_execution_helpers.hpp"

using namespace ::testing;  // NOLINT

class InfoEndToEndTestFixture : public Test
{
public:
  InfoEndToEndTestFixture()
  {
    database_path_ = _SRC_RESOURCES_DIR_PATH;  // variable defined in CMakeLists.txt
  }

  std::string database_path_;
};

TEST_F(InfoEndToEndTestFixture, info_end_to_end_test) {
  internal::CaptureStdout();
  auto exit_code = execute_and_wait_until_completion("ros2 bag info cdr_test", database_path_);
  std::string output = internal::GetCapturedStdout();

  EXPECT_THAT(exit_code, Eq(EXIT_SUCCESS));
  // The bag size depends on the os and is not asserted, the time is asserted time zone independent
  EXPECT_THAT(
    output, ContainsRegex(
      "\nFiles:             cdr_test_0\\.db3"
      "\nBag size:          .*B"
      "\nStorage id:        sqlite3"
      "\nDuration:          0\\.151s"
      "\nStart:             Apr  9 2020 .*:.*:36.763 \\(1586406456\\.763\\)"
      "\nEnd                Apr  9 2020 .*:.*:36.914 \\(1586406456\\.914\\)"
      "\nMessages:          7"
      "\nTopic information: "));
  EXPECT_THAT(
    output, HasSubstr(
      "Topic: /test_topic | Type: test_msgs/msg/BasicTypes | Count: 3 | "
      "Serialization Format: cdr\n"));
  EXPECT_THAT(
    output, HasSubstr(
      "Topic: /array_topic | Type: test_msgs/msg/Arrays | Count: 4 | "
      "Serialization Format: cdr"));
}

// TODO(Martin-Idel-SI): Revisit exit code non-zero here, gracefully should be exit code zero
TEST_F(InfoEndToEndTestFixture, info_fails_gracefully_if_bag_does_not_exist) {
  internal::CaptureStderr();
  auto exit_code =
    execute_and_wait_until_completion("ros2 bag info does_not_exist", database_path_);
  auto error_output = internal::GetCapturedStderr();

  EXPECT_THAT(exit_code, Eq(EXIT_FAILURE));
  EXPECT_THAT(error_output, HasSubstr("'does_not_exist' does not exist"));
}

TEST_F(InfoEndToEndTestFixture, info_fails_gracefully_if_metadata_yaml_file_does_not_exist) {
  internal::CaptureStderr();
  auto exit_code =
    execute_and_wait_until_completion("ros2 bag info " + database_path_, database_path_);
  auto error_output = internal::GetCapturedStderr();

  EXPECT_THAT(exit_code, Eq(EXIT_SUCCESS));
  EXPECT_THAT(error_output, HasSubstr("Could not read metadata for " + database_path_));
}
