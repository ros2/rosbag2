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

#ifndef ROSBAG2_TEST_COMMON__TEMPORARY_DIRECTORY_FIXTURE_HPP_
#define ROSBAG2_TEST_COMMON__TEMPORARY_DIRECTORY_FIXTURE_HPP_

#include <gmock/gmock.h>

#include <iostream>
#include <string>

#ifdef _WIN32
# include <direct.h>
# include <Windows.h>
#else
# include <unistd.h>
# include <sys/types.h>
# include <dirent.h>
#endif

using namespace ::testing;  // NOLINT

namespace rosbag2_test_common
{

class TemporaryDirectoryFixture : public Test
{
public:
  TemporaryDirectoryFixture()
  {
    char template_char[] = "tmp_test_dir.XXXXXX";
#ifdef _WIN32
    char temp_path[255];
    GetTempPathA(255, temp_path);
    _mktemp_s(template_char, strnlen(template_char, 20) + 1);
    temporary_dir_path_ = std::string(temp_path) + std::string(template_char);
    _mkdir(temporary_dir_path_.c_str());
#else
    char * dir_name = mkdtemp(template_char);
    temporary_dir_path_ = dir_name;
#endif
  }

  ~TemporaryDirectoryFixture() override
  {
    remove_directory_recursively(temporary_dir_path_);
  }

  void remove_directory_recursively(const std::string & directory_path)
  {
#ifdef _WIN32
    // We need a string of type PCZZTSTR, which is a double null terminated char ptr
    size_t length = strlen(directory_path.c_str());
    TCHAR * temp_dir = new TCHAR[length + 2];
    memcpy(temp_dir, temporary_dir_path_.c_str(), length);
    temp_dir[length] = 0;
    temp_dir[length + 1] = 0;  // double null terminated

    SHFILEOPSTRUCT file_options;
    file_options.hwnd = nullptr;
    file_options.wFunc = FO_DELETE;  // delete (recursively)
    file_options.pFrom = temp_dir;
    file_options.pTo = nullptr;
    file_options.fFlags = FOF_NOCONFIRMATION | FOF_SILENT;  // do not prompt user
    file_options.fAnyOperationsAborted = FALSE;
    file_options.lpszProgressTitle = nullptr;
    file_options.hNameMappings = nullptr;

    const auto rc = SHFileOperation(&file_options);
    if (0 != rc) {
      std::cerr << "Failed to recursively delete '" << directory_path <<
        "' Error code: " << rc << std::endl;
    }

    if (file_options.fAnyOperationsAborted) {
      std::cerr << "Recursive delete of '" << directory_path <<
        "' was aborted." << std::endl;
    }
    delete[] temp_dir;
#else
    DIR * dir = opendir(directory_path.c_str());
    if (!dir) {
      return;
    }
    struct dirent * directory_entry;
    while ((directory_entry = readdir(dir)) != nullptr) {
      // Make sure to not call ".." or "." entries in directory (might delete everything)
      if (strcmp(directory_entry->d_name, ".") != 0 && strcmp(directory_entry->d_name, "..") != 0) {
        if (directory_entry->d_type == DT_DIR) {
          remove_directory_recursively(directory_path + "/" + directory_entry->d_name);
        }
        remove((directory_path + "/" + directory_entry->d_name).c_str());
      }
    }
    closedir(dir);
    remove(temporary_dir_path_.c_str());
#endif
  }

  std::string temporary_dir_path_;
};

}  // namespace rosbag2_test_common

#endif  // ROSBAG2_TEST_COMMON__TEMPORARY_DIRECTORY_FIXTURE_HPP_
