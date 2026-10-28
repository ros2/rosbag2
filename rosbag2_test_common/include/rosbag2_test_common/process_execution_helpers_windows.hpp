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
#include <iostream>
#include <string>
#include <thread>

using namespace ::testing;  // NOLINT

struct Process
{
  PROCESS_INFORMATION process_info;
  HANDLE job_handle;
};
using ProcessHandle = Process;

namespace
{
std::string last_error_string(DWORD error = GetLastError())
{
  char * buffer = nullptr;
  DWORD length = FormatMessageA(
    FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
    nullptr, error, 0, reinterpret_cast<LPSTR>(&buffer), 0, nullptr);
  std::string result = "error " + std::to_string(error);
  if (length != 0 && buffer != nullptr) {
    result += ": ";
    result.append(buffer, length);
    LocalFree(buffer);
  }
  while (!result.empty() && (result.back() == '\n' || result.back() == '\r')) {
    result.pop_back();
  }
  return result;
}

BOOL WINAPI ignore_console_ctrl_events(DWORD)
{
  return TRUE;
}

void make_inheritable(HANDLE handle)
{
  if (handle != nullptr && handle != INVALID_HANDLE_VALUE) {
    SetHandleInformation(handle, HANDLE_FLAG_INHERIT, HANDLE_FLAG_INHERIT);
  }
}
}  // namespace

/// Create a process running `command`.
/**
 * With `own_console` the child is created suspended, with its own hidden console
 * and its own process group, so that stop_execution() can deliver a console
 * control event to it.  The caller must resume its main thread.  The child
 * still inherits our standard handles, so its output ends up in the test log.
 *
 * The console the CI jobs run under does not support GenerateConsoleCtrlEvent at
 * all ("Incorrect function"), see ros2/rosbag2#1342.  A console created for the
 * child does, which is why the child gets one of its own.
 */
PROCESS_INFORMATION create_process(
  TCHAR * command, const char * path = nullptr, bool own_console = false)
{
  STARTUPINFO start_up_info{};
  start_up_info.cb = sizeof(start_up_info);
  PROCESS_INFORMATION process_info{};
  DWORD creation_flags = 0;
  BOOL inherit_handles = FALSE;
  if (own_console) {
    creation_flags = CREATE_NO_WINDOW | CREATE_NEW_PROCESS_GROUP | CREATE_SUSPENDED;
    start_up_info.dwFlags = STARTF_USESTDHANDLES;
    start_up_info.hStdInput = GetStdHandle(STD_INPUT_HANDLE);
    start_up_info.hStdOutput = GetStdHandle(STD_OUTPUT_HANDLE);
    start_up_info.hStdError = GetStdHandle(STD_ERROR_HANDLE);
    make_inheritable(start_up_info.hStdInput);
    make_inheritable(start_up_info.hStdOutput);
    make_inheritable(start_up_info.hStdError);
    inherit_handles = TRUE;
  }

  BOOL created = CreateProcess(
    nullptr,
    command,
    nullptr,
    nullptr,
    inherit_handles,
    creation_flags,
    nullptr,
    path,
    &start_up_info,
    &process_info);
  EXPECT_TRUE(created) << "CreateProcess(" << command << ") failed: " << last_error_string();

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

  // Created suspended so that the child cannot spawn its own children (e.g. the
  // ros2 launcher starting python) before it is in the job.
  auto process_info = create_process(command_char, nullptr, true);

  EXPECT_TRUE(AssignProcessToJobObject(h_job, process_info.hProcess)) <<
    "AssignProcessToJobObject failed: " << last_error_string();
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
  EXPECT_NE(exit_code, STILL_ACTIVE) << "Testing process " <<
    handle.process_info.dwProcessId << " hangout.\n";
  if (exit_code != STILL_ACTIVE) {
    EXPECT_EQ(exit_code, 0u) << "exit code = " << exit_code;
  }
  return exit_code != STILL_ACTIVE;
}

/// @brief Stop a process started with start_execution() if it is still running
/// @param handle Process handle returned from start_execution(command)
/// @param signum Accepted for parity with the Unix version.  Windows has no SIGINT or
/// SIGTERM delivery between processes; both are sent as a CTRL_BREAK_EVENT, which the
/// child sees as SIGBREAK.
/// @param timeout Timeout in fraction of seconds
void stop_execution(
  const ProcessHandle & handle,
  int signum = SIGINT,
  std::chrono::duration<double> timeout = std::chrono::seconds(10))
{
  (void)signum;
  DWORD exit_code = STILL_ACTIVE;
  EXPECT_TRUE(GetExitCodeProcess(handle.process_info.hProcess, &exit_code));
  if (exit_code == STILL_ACTIVE) {
    // The child has its own console (see create_process).  Join it to deliver the
    // event to the child's process group, which does not include this process.
    // CTRL_BREAK_EVENT is the only event that can be addressed to a group other than
    // our own; CTRL_C_EVENT is disabled in a group created with CREATE_NEW_PROCESS_GROUP.
    SetConsoleCtrlHandler(ignore_console_ctrl_events, TRUE);
    FreeConsole();
    bool attached = AttachConsole(handle.process_info.dwProcessId);
    EXPECT_TRUE(attached) << "AttachConsole failed: " << last_error_string();
    if (attached) {
      EXPECT_TRUE(GenerateConsoleCtrlEvent(CTRL_BREAK_EVENT, handle.process_info.dwProcessId)) <<
        "GenerateConsoleCtrlEvent failed: " << last_error_string();
    }
    bool process_finished = wait_until_completion(handle, timeout);
    FreeConsole();
    AttachConsole(ATTACH_PARENT_PROCESS);
    SetConsoleCtrlHandler(ignore_console_ctrl_events, FALSE);
    if (!process_finished) {
      std::cerr << "Testing process " << handle.process_info.dwProcessId <<
        " hangout. Killing it with TerminateProcess(..) \n";
      EXPECT_TRUE(TerminateProcess(handle.process_info.hProcess, 2));
    }
  }
  close_process_handles(handle.process_info);
  CloseHandle(handle.job_handle);
}

#endif  // ROSBAG2_TEST_COMMON__PROCESS_EXECUTION_HELPERS_WINDOWS_HPP_
