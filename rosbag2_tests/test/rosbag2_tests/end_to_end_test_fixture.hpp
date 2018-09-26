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

#ifndef END_TO_END_TEST_FIXTURE_HPP_
#define END_TO_END_TEST_FIXTURE_HPP_

#include <gmock/gmock.h>

#include <string>
#include <thread>

#ifdef _WIN32
# include <direct.h>
# include <Windows.h>
#endif

using namespace ::testing;  // NOLINT

class EndToEndTestFixture : public Test
{
public:
  EndToEndTestFixture()
  {
    database_path_ = _SRC_RESOURCES_DIR_PATH;  // variable defined in CMakeLists.txt
  }

  void execute(const std::string & command)
  {
#ifdef _WIN32
    size_t length = strlen(command.c_str());
    TCHAR * command_char = new TCHAR[length + 1];
    memcpy(command_char, command.c_str(), length + 1);

    STARTUPINFO start_up_info{};
    PROCESS_INFORMATION process_info{};
    CreateProcess(
      nullptr,
      command_char,
      nullptr,
      nullptr,
      false,
      0,
      nullptr,
      database_path_.c_str(),
      &start_up_info,
      &process_info);
    DWORD exit_code = 259;  // 259 is the code one gets if the proces is still active.
    while (exit_code == 259) {
      GetExitCodeProcess(process_info.hProcess, &exit_code);
    }
    CloseHandle(process_info.hProcess);
    CloseHandle(process_info.hThread);
    EXPECT_THAT(exit_code, Eq(0));
    delete[] command_char;
#else
    chdir(database_path_.c_str());
    int exit_code = system(command.c_str());
    EXPECT_THAT(exit_code, Eq(0));
#endif
  }

  std::string database_path_;
};

#endif  // END_TO_END_TEST_FIXTURE_HPP_
