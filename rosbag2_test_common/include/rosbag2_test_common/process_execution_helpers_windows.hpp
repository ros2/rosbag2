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
#include <windows.h>

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
    // Create process suspended and resume it after adding to the newly created job. Otherwise,
    // there is a potential race condition where newly created process starts a subprocess
    // before we've called AssignProcessToJobObject();
    CREATE_NEW_PROCESS_GROUP | CREATE_SUSPENDED,
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
  ResumeThread(process.hThread);
  DWORD exit_code = STILL_ACTIVE;
  while (exit_code == STILL_ACTIVE) {
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
  ResumeThread(process_info.hThread);
  Process process;
  process.process_info = process_info;
  process.job_handle = h_job;

  delete[] command_char;
  return process;
}

/// @brief Wait for process to finish with timeout
/// @param handle Process handle returned from start_execution(command)
/// @param timeout Timeout in fraction of seconds
/// @return true if process has finished during timeout and false if timeout was reached and
/// process is still running
bool wait_until_completion(
  const ProcessHandle & handle,
  std::chrono::duration<double> timeout = std::chrono::seconds(10))
{
  DWORD exit_code = 0;
  std::chrono::steady_clock::time_point const start = std::chrono::steady_clock::now();
  EXPECT_TRUE(GetExitCodeProcess(handle.process_info.hProcess, &exit_code));
  while (exit_code == STILL_ACTIVE && std::chrono::steady_clock::now() - start < timeout) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    EXPECT_TRUE(GetExitCodeProcess(handle.process_info.hProcess, &exit_code));
  }
  EXPECT_EQ(exit_code, 0);
  return exit_code != STILL_ACTIVE;
}

/// @brief Force to stop process with signal if it's currently running
/// @param handle Process handle returned from start_execution(command)
/// @param signum Not uses for Windows version
/// @param timeout Timeout in fraction of seconds
void stop_execution(
  const ProcessHandle & handle,
  int signum = SIGINT,
  std::chrono::duration<double> timeout = std::chrono::seconds(10))
{
  // Match the Unix version by allowing for int signum argument - however Windows does not have
  // Linux signals in the same way, so there isn't a 1:1 mapping to dispatch e.g. SIGINT or SIGTERM
  DWORD exit_code = STILL_ACTIVE;
  EXPECT_TRUE(GetExitCodeProcess(handle.process_info.hProcess, &exit_code));
  // Make sure that the process is still running properly before stopping it.
  if (exit_code == STILL_ACTIVE) {
    switch (signum) {
      // According to the
      // https://learn.microsoft.com/en-us/cpp/c-runtime-library/reference/signal?view=msvc-170
      // SIGINT and SIGBREAK is not supported for any Win32 application.
      // Need to use native Windows control event instead.
      case SIGINT:
        EXPECT_TRUE(GenerateConsoleCtrlEvent(CTRL_C_EVENT, handle.process_info.dwProcessId));
        break;
      case SIGBREAK:
        EXPECT_TRUE(GenerateConsoleCtrlEvent(CTRL_BREAK_EVENT, handle.process_info.dwProcessId));
        break;
      case SIGTERM:
      // The CTRL_CLOSE_EVENT is analog of the SIGTERM from POSIX. Windows sends CTRL_CLOSE_EVENT
      // to all processes attached to a console when the user closes the console (either by
      // clicking Close on the console window's window menu, or by clicking the End Task
      // button command from Task Manager). Although according to the
      // https://learn.microsoft.com/en-us/windows/console/generateconsolectrlevent the
      // GenerateConsoleCtrlEvent doesn't support sending CTRL_CLOSE_EVENT. There are no way to
      // directly send CTRL_CLOSE_EVENT to the process in the same console application.
      // Therefore, adding SIGTERM to the unsupported events.
      default:
        throw std::runtime_error("Unsupported signum: " + std::to_string(signum));
    }
    bool process_finished = wait_until_completion(handle, timeout);
    if (!process_finished) {
      std::cerr << "Testing process " << handle.process_info.dwProcessId <<
        " hangout. Killing it with TerminateProcess(..) \n";
      EXPECT_TRUE(TerminateProcess(handle.process_info.hProcess, 2));
      EXPECT_TRUE(process_finished);
    }
  }
  close_process_handles(handle.process_info);
  CloseHandle(handle.job_handle);
}

#endif  // ROSBAG2_TEST_COMMON__PROCESS_EXECUTION_HELPERS_WINDOWS_HPP_
