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

#ifndef ROSBAG2_TEST_COMMON__PROCESS_EXECUTION_HELPERS_WINDOWS_HPP_
#define ROSBAG2_TEST_COMMON__PROCESS_EXECUTION_HELPERS_WINDOWS_HPP_

#include <gmock/gmock.h>

#include <direct.h>
#include <Windows.h>

#include <chrono>
#include <csignal>
#include <cstdlib>
#include <string>
#include <thread>

using namespace ::testing;  // NOLINT

struct Process
{
  PROCESS_INFORMATION process_info;
  HANDLE job_handle;
};
using ProcessHandle = Process;

PROCESS_INFORMATION create_process(TCHAR * command, const char * path = nullptr)
{
  STARTUPINFO start_up_info{};
  PROCESS_INFORMATION process_info{};

  CreateProcess(
    nullptr,
    command,
    nullptr,
    nullptr,
    false,
    0,
    nullptr,
    path,
    &start_up_info,
    &process_info);

  return process_info;
}

void close_process_handles(const PROCESS_INFORMATION & process)
{
  CloseHandle(process.hProcess);
  CloseHandle(process.hThread);
}

void const_char_to_tchar(const char * source, TCHAR * destination)
{
  size_t length = strlen(source);
  memcpy(destination, source, length + 1);
}

int execute_and_wait_until_completion(const std::string & command, const std::string & path)
{
  TCHAR * command_char = new TCHAR[strlen(command.c_str()) + 1];
  const_char_to_tchar(command.c_str(), command_char);

  auto process = create_process(command_char, path.c_str());
  DWORD exit_code = 259;  // 259 is the code one gets if the process is still active.
  while (exit_code == 259) {
    EXPECT_TRUE(GetExitCodeProcess(process.hProcess, &exit_code));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  close_process_handles(process);

  delete[] command_char;
  return static_cast<int>(exit_code);
}

ProcessHandle start_execution(const std::string & command)
{
  auto h_job = CreateJobObject(nullptr, nullptr);
  JOBOBJECT_EXTENDED_LIMIT_INFORMATION info{};
  info.BasicLimitInformation.LimitFlags = JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE;
  SetInformationJobObject(h_job, JobObjectExtendedLimitInformation, &info, sizeof(info));

  TCHAR * command_char = new TCHAR[strlen(command.c_str()) + 1];
  const_char_to_tchar(command.c_str(), command_char);

  auto process_info = create_process(command_char);

  AssignProcessToJobObject(h_job, process_info.hProcess);
  Process process;
  process.process_info = process_info;
  process.job_handle = h_job;

  delete[] command_char;
  return process;
}

/// @brief Wait for process to finish with timeout
/// @param process_id Process ID
/// @param timeout Timeout in fraction of seconds
/// @return true if process has finished during timeout and false if timeout was reached and
/// process is still running
bool wait_until_completion(
  const ProcessHandle & process_id,
  std::chrono::duration<double> timeout = std::chrono::seconds(5))
{
  DWORD exit_code = 0;
  std::chrono::steady_clock::time_point const start = std::chrono::steady_clock::now();
  EXPECT_TRUE(GetExitCodeProcess(handle.process_info.hProcess, &exit_code));
  // 259 indicates that the process is still active
  while (exit_code == 259 && std::chrono::steady_clock::now() - start < timeout) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    EXPECT_TRUE(GetExitCodeProcess(handle.process_info.hProcess, &exit_code));
  }
  return exit_code != 259;
}

/// @brief Force to stop process with signal if it's currently running
/// @param process_id Process ID
/// @param signum Not uses for Windows version
/// @param timeout Timeout in fraction of seconds
void stop_execution(
  const ProcessHandle & handle,
  int signum = SIGINT,
  std::chrono::duration<double> timeout = std::chrono::seconds(10))
{
  // Match the Unix version by allowing for int signum argument - however Windows does not have
  // Linux signals in the same way, so there isn't a 1:1 mapping to dispatch e.g. SIGTERM
  DWORD exit_code;
  EXPECT_TRUE(GetExitCodeProcess(handle.process_info.hProcess, &exit_code));
  // 259 indicates that the process is still active: we want to make sure that the process is
  // still running properly before killing it.
  if (exit_code == 259) {
    EXPECT_TRUE(GenerateConsoleCtrlEvent(CTRL_C_EVENT, handle.process_info.dwThreadId));
    bool process_finished = wait_until_completion(timeout);
    if (!process_finished) {
      std::cerr << "Testing process " << handle.process_info.hProcess <<
        " hangout. Killing it with TerminateProcess(..) \n";
      EXPECT_TRUE(TerminateProcess(handle.process_info.hProcess, 2));
      EXPECT_TRUE(process_finished);
    }
  }
  close_process_handles(handle.process_info);
  CloseHandle(handle.job_handle);
}

#endif  // ROSBAG2_TEST_COMMON__PROCESS_EXECUTION_HELPERS_WINDOWS_HPP_
