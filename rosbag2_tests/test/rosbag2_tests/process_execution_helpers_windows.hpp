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

#ifndef ROSBAG2_TESTS__PROCESS_EXECUTION_HELPERS_WINDOWS_HPP_
#define ROSBAG2_TESTS__PROCESS_EXECUTION_HELPERS_WINDOWS_HPP_

#include <gmock/gmock.h>

#include <direct.h>
#include <Windows.h>

#include <chrono>
#include <cstdlib>
#include <string>

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
    GetExitCodeProcess(process.hProcess, &exit_code);
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

void stop_execution(const ProcessHandle & handle)
{
  DWORD exit_code;
  GetExitCodeProcess(handle.process_info.hProcess, &exit_code);
  // 259 indicates that the process is still active: we want to make sure that the process is
  // still running properly before killing it.
  EXPECT_THAT(exit_code, Eq(259));

  close_process_handles(handle.process_info);
  CloseHandle(handle.job_handle);
}

#endif  // ROSBAG2_TESTS__PROCESS_EXECUTION_HELPERS_WINDOWS_HPP_
