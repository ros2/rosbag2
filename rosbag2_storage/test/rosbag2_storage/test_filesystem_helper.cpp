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

#include <fstream>
#include <string>

#include "rosbag2_storage/filesystem_helper.hpp"
#include "rosbag2_test_common/temporary_directory_fixture.hpp"

using namespace ::testing;  // NOLINT
using namespace rosbag2_storage;  // NOLINT
using namespace rosbag2_test_common;  // NOLINT

class FilesystemHelperFixture : public TemporaryDirectoryFixture {};

TEST_F(FilesystemHelperFixture, calculate_directory_size_adds_size_of_two_directories)
{
  std::ofstream out(FilesystemHelper::concat({temporary_dir_path_, "file1.txt"}));
  out << "test";
  out.close();

  std::ofstream out2(FilesystemHelper::concat({temporary_dir_path_, "file2.txt"}));
  out2 << "something";
  out2.close();

  size_t size = FilesystemHelper::calculate_directory_size(temporary_dir_path_);

  EXPECT_THAT(size, Eq(13u));
}

TEST_F(FilesystemHelperFixture, file_exists_shows_whether_file_exists)
{
  std::ofstream out(FilesystemHelper::concat({temporary_dir_path_, "file1.txt"}));
  out << "test";
  out.close();

  EXPECT_TRUE(FilesystemHelper::file_exists(
      FilesystemHelper::concat({temporary_dir_path_, "file1.txt"})));
  EXPECT_FALSE(FilesystemHelper::file_exists(
      FilesystemHelper::concat({temporary_dir_path_, "file2.txt"})));
}

TEST(FilesystemHelper, concat_joins_string_list_with_directory_separators)
{
  auto sep = std::string(FilesystemHelper::separator);

  auto path = FilesystemHelper::concat({"some", "path", "to", "a", "folder"});

  EXPECT_THAT(path, Eq("some" + sep + "path" + sep + "to" + sep + "a" + sep + "folder"));
}

TEST(FilesystemHelper, concat_returns_empty_string_for_empty_list)
{
  auto path = FilesystemHelper::concat({});

  EXPECT_THAT(path, Eq(""));
}

TEST(FilesystemHelper, get_folder_name_for_path_ending_with_separator)
{
  auto sep = std::string(FilesystemHelper::separator);
  auto path = FilesystemHelper::concat({"some", "path", "to", "a", "folder"}) + sep;

  auto folder_name = FilesystemHelper::get_folder_name(path);

  EXPECT_THAT(folder_name, Eq("folder"));
}

TEST(FilesystemHelper, get_folder_name_for_path_not_ending_with_separator)
{
  auto path = FilesystemHelper::concat({"some", "path", "to", "a", "folder"});

  auto folder_name = FilesystemHelper::get_folder_name(path);

  EXPECT_THAT(folder_name, Eq("folder"));
}

TEST(FilesystemHelper, get_file_name_returns_last_part_of_path)
{
  auto path = FilesystemHelper::concat({"some", "path", "to", "a", "file.txt"});

  auto folder_name = FilesystemHelper::get_file_name(path);

  EXPECT_THAT(folder_name, Eq("file.txt"));
}

TEST(FilesystemHelper, get_file_name_returns_empty_string_if_passed_a_path_with_trailing_separator)
{
  auto sep = std::string(FilesystemHelper::separator);
  auto path = FilesystemHelper::concat({"some", "path", "to", "a", "folder"}) + sep;

  auto folder_name = FilesystemHelper::get_file_name(path);

  EXPECT_THAT(folder_name, Eq(""));
}
