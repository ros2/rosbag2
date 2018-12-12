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

#ifndef ROSBAG2_TESTS__PROCESS_EXECUTION_HELPERS_UNIX_HPP_
#define ROSBAG2_TESTS__PROCESS_EXECUTION_HELPERS_UNIX_HPP_

#include <gmock/gmock.h>

#include <signal.h>
#include <stdlib.h>

#include <chrono>
#include <cstdlib>
#include <string>

using namespace ::testing;  // NOLINT

using ProcessHandle = int;

int execute_and_wait_until_completion(const std::string & command, const std::string & path)
{
  char previous_dir[PATH_MAX];
  auto ret_get_cwd = getcwd(previous_dir, PATH_MAX);
  if (ret_get_cwd == NULL) {
    return EXIT_FAILURE;
  }

  auto ret_ch_dir = chdir(path.c_str());
  if (ret_ch_dir != 0) {
    return EXIT_FAILURE;
  }
  auto exitcode = std::system(command.c_str());
  ret_ch_dir = chdir(previous_dir);
  if (ret_ch_dir != 0) {
    return EXIT_FAILURE;
  }

  return WEXITSTATUS(exitcode);
}

ProcessHandle start_execution(const std::string & command)
{
  auto process_id = fork();
  if (process_id == 0) {
    setpgid(0, 0);
    execl("/bin/sh", "sh", "-c", command.c_str(), static_cast<char *>(nullptr));
  }
  return process_id;
}

void stop_execution(const ProcessHandle & handle)
{
  killpg(handle, SIGINT);
  int child_return_code;
  waitpid(handle, &child_return_code, 0);
  // this call will make sure that the process does execute without issues before it is killed by
  // the user in the test or, in case it runs until completion, that it has correctly executed.
  EXPECT_THAT(WEXITSTATUS(child_return_code), Not(Eq(EXIT_FAILURE)));
}

#endif  // ROSBAG2_TESTS__PROCESS_EXECUTION_HELPERS_UNIX_HPP_
