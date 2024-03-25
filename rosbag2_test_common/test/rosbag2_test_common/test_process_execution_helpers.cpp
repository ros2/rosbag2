// Copyright 2023 Apex.AI, Inc. or its affiliates. All Rights Reserved.
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

#include "rosbag2_test_common/process_execution_helpers.hpp"
#include "rcpputils/scope_exit.hpp"

class ProcessExecutionHelpersTest : public ::testing::Test
{
public:
  ProcessExecutionHelpersTest() = default;
};

TEST_F(ProcessExecutionHelpersTest, ctrl_c_event_can_be_send_and_received) {
  testing::internal::CaptureStdout();
  auto process_id = start_execution("loop_with_ctrl_c_handler");
  auto cleanup_process_handle = rcpputils::make_scope_exit(
    [process_id]() {
      stop_execution(process_id);
    });

  // Sleep for 1 second to yield CPU resources to the newly spawned process, to make sure that
  // signal handlers has been installed.
  std::this_thread::sleep_for(std::chrono::seconds(1));

  std::string test_output = testing::internal::GetCapturedStdout();
  EXPECT_THAT(test_output, HasSubstr("Waiting in a loop for CTRL+C event"));

  // Send SIGINT to child process and check exit code
  stop_execution(process_id, SIGINT);
  cleanup_process_handle.cancel();
}
