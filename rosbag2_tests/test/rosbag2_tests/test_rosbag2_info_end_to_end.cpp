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
  auto exit_code = execute_and_wait_until_completion("ros2 bag info test", database_path_);
  std::string output = internal::GetCapturedStdout();

  EXPECT_THAT(exit_code, Eq(EXIT_SUCCESS));
  // We have two asserts because the bag size depends on the os and, therefore, cannot be asserted.
  EXPECT_THAT(output, HasSubstr("\nFiles:            test.db3\n"));
  EXPECT_THAT(output, HasSubstr(
      "\nStorage id:       sqlite3"
      "\nStorage format:   cdr"
      "\nDuration:         0.155s"
      "\nStart:            Sep 18 2018 16:56:44.241 (1537282604.241)"
      "\nEnd               Sep 18 2018 16:56:44.397 (1537282604.397)"
      "\nMessages:         7"
      "\nTopics with Type: /test_topic; test_msgs/Primitives; 3 msgs"
      "\n                  /array_topic; test_msgs/StaticArrayPrimitives; 4 msgs"));
}
