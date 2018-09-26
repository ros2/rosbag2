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

#include <string>
#include <thread>

#ifdef _WIN32
# include <direct.h>
# include <Windows.h>
#endif

using namespace ::testing;  // NOLINT
using namespace std::chrono_literals;  // NOLINT

class InfoEndToEndTestFixture : public Test
{
public:
  InfoEndToEndTestFixture()
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
    CloseHandle(process_info.hProcess);
    CloseHandle(process_info.hThread);
    delete[] command_char;
#else
    chdir(database_path_.c_str());
    system(command.c_str());
#endif
  }

  std::string database_path_;
};

TEST_F(InfoEndToEndTestFixture, info_end_to_end_test) {
  internal::CaptureStdout();
  execute("ros2 bag info test.bag");
  // We wait before retrieving the captured stdout to make sure that the info have been printed.
  std::this_thread::sleep_for(2s);
  std::string output = internal::GetCapturedStdout();

  // TODO(botteroa-si): update once correct pretty printing of baginfo is available.
  EXPECT_THAT(output, HasSubstr("printing bag info of 'test.bag'..."));
}
