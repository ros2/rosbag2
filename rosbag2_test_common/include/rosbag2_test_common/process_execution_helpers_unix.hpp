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

#ifndef ROSBAG2_TEST_COMMON__PROCESS_EXECUTION_HELPERS_UNIX_HPP_
#define ROSBAG2_TEST_COMMON__PROCESS_EXECUTION_HELPERS_UNIX_HPP_

#include <gmock/gmock.h>

#include <signal.h>
#include <stdlib.h>
#include <thread>

#include <chrono>
#include <cstdlib>
#include <vector>
#include <string>

using namespace ::testing;  // NOLINT

using ProcessHandle = int;

/// \brief Split command string on words separated by spaces for further usage with execvp(..)
/// \param [in] command - command line string with arguments split by spaces
/// \return vector of char pointers
std::vector<char *> command_string_to_arguments(const std::string & command)
{
  std::vector<char *> arguments;
  std::istringstream ss(command);
  std::string token;
  while (std::getline(ss, token, ' ')) {
    // dup strings to get non-const, should be free'd after execvp
    if (!token.empty()) {  // Skipping extra spaces between arguments
      arguments.push_back(strdup(token.c_str()));
    }
  }
  arguments.push_back(nullptr);  // explicit nullptr to tell execvp where to stop
  return arguments;
}

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
  auto status = std::system(command.c_str());
  ret_ch_dir = chdir(previous_dir);
  if (ret_ch_dir != 0) {
    return EXIT_FAILURE;
  }
  EXPECT_EQ(WIFEXITED(status), true) << "status = " << status;
  if (WIFSIGNALED(status)) {
    // Process terminated by signal
    return WTERMSIG(status);
  }
  return WEXITSTATUS(status);
}

ProcessHandle start_execution(const std::string & command)
{
  auto process_id = fork();
  if (process_id == -1) {
    throw std::runtime_error("Failed to start process via fork()");
  }
  if (process_id == 0) {  // In child process
    // Split command on words separated by spaces to further use in execvp
    // Note alternative approach with execl("/bin/sh", "sh", "-c", command.c_str(), nullptr);
    // will spawn one more child process with default SIG_INT handler and will incorrectly return
    // exit codes in case of signal handling.
    std::vector<char *> arguments{command_string_to_arguments(command)};
    const char * cmd = arguments[0];
    int ret = execvp(cmd, arguments.data());
    for (auto str : arguments) {
      free(str);
      str = nullptr;
    }
    if (ret == -1) {
      throw std::runtime_error("Failed to call execvp(cmd, args)");
    }
  }
  return process_id;
}

/// @brief Wait for process to finish with timeout
/// @param process_id Process ID
/// @param timeout Timeout in fraction of seconds
/// @return true if process has finished during timeout and false if timeout was reached and
/// process is still running
bool wait_until_completion(
  const ProcessHandle & process_id,
  std::chrono::duration<double> timeout = std::chrono::seconds(10))
{
  pid_t wait_ret_code = 0;
  int status = EXIT_FAILURE;
  std::chrono::steady_clock::time_point const start = std::chrono::steady_clock::now();
  // Wait for process to finish with timeout
  while (wait_ret_code == 0 && std::chrono::steady_clock::now() - start < timeout) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    // WNOHANG - wait for processes without causing the caller to be blocked
    wait_ret_code = waitpid(process_id, &status, WNOHANG);
  }
  EXPECT_NE(wait_ret_code, -1);
  EXPECT_EQ(wait_ret_code, process_id) << "status = " << status;
  return wait_ret_code != 0;
}

/// @brief Force to stop process with signal if it's currently running
/// @param process_id Process ID
/// @param signum Signal to use for stopping process. The default is SIGINT.
/// @param timeout Timeout in fraction of seconds
void stop_execution(
  const ProcessHandle & process_id,
  int signum = SIGINT,
  std::chrono::duration<double> timeout = std::chrono::seconds(10))
{
  if (kill(process_id, 0) == 0) {
    // If process is still running, then send signal to stop it and check return code
    EXPECT_NE(kill(process_id, signum), -1) << "Failed to send signal " << signum <<
      " to process: " << process_id;
    int status = EXIT_FAILURE;
    pid_t wait_ret_code = 0;
    std::chrono::steady_clock::time_point const start = std::chrono::steady_clock::now();
    // Wait for process to finish with timeout
    while (wait_ret_code == 0 && std::chrono::steady_clock::now() - start < timeout) {
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
      // WNOHANG - wait for processes without causing the caller to be blocked
      wait_ret_code = waitpid(process_id, &status, WNOHANG);
    }
    if (wait_ret_code == 0) {
      std::cerr << "Testing process " << process_id << " hangout. Killing it with SIGKILL \n";
      kill(process_id, SIGKILL);
    }
    // Make sure that the process does execute without issues before it is killed by
    // the user in the test or, in case it runs until completion, that it has correctly executed.
    EXPECT_NE(wait_ret_code, -1);
    EXPECT_EQ(wait_ret_code, process_id);
    EXPECT_EQ(WIFEXITED(status), true) << "status = " << status;
    EXPECT_EQ(WIFSIGNALED(status), false) << "Process terminated by signal: " << WTERMSIG(status);
    EXPECT_EQ(WEXITSTATUS(status), EXIT_SUCCESS) << "status = " << status;
  }
}

#endif  // ROSBAG2_TEST_COMMON__PROCESS_EXECUTION_HELPERS_UNIX_HPP_
