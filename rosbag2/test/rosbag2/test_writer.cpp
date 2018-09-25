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

#include <chrono>
#include <fstream>
#include <string>
#include <thread>

#include "rosbag2_storage/filesystem_helpers.hpp"
#include "rosbag2/storage_options.hpp"
#include "rosbag2/writer.hpp"
#include "temporary_directory_fixture.hpp"

using namespace ::testing;  // NOLINT
using namespace std::chrono_literals;  // NOLINT

void write_file(const std::string & uri)
{
  rosbag2::StorageOptions options = {uri, "sqlite3"};
  auto writer = rosbag2::create_default_writer();
  writer->open(options);
  writer->create_topic({"/string_topic", "topic/Type"});
}

std::string get_yaml_content(const std::string & filename)
{
  std::ifstream read_file(filename);
  std::ostringstream ss;
  ss << read_file.rdbuf();
  return ss.str();
}

TEST_F(TemporaryDirectoryFixture, writer_writes_correct_yaml_at_shutdown) {
  write_file(temporary_dir_path_);

  std::string expected_start("rosbag2_bagfile_information:"
    "\n  storage_identifier: sqlite3"
    "\n  encoding: cdr");

  std::string actual_yaml = get_yaml_content(
    temporary_dir_path_ + rosbag2_storage::separator() + "metadata.yaml");
  ASSERT_THAT(actual_yaml, StartsWith(expected_start));
}
