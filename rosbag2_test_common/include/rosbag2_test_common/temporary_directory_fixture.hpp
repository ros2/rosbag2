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

#include <string>

#include "rcpputils/filesystem_helper.hpp"

using namespace ::testing;  // NOLINT

namespace rosbag2_test_common
{

class TemporaryDirectoryFixture : public Test
{
public:
  TemporaryDirectoryFixture()
  {
    auto tmpdir = rcpputils::fs::temp_directory_path();
    temporary_dir_path_ = rcpputils::fs::create_temp_directory(
      rcpputils::fs::temp_directory_path(),
      "tmp_test_dir").string();
  }

  ~TemporaryDirectoryFixture() override
  {
    rcpputils::fs::remove_all(rcpputils::fs::path(temporary_dir_path_));
  }

  std::string temporary_dir_path_;
};

}  // namespace rosbag2_test_common

#endif  // ROSBAG2_TEST_COMMON__TEMPORARY_DIRECTORY_FIXTURE_HPP_
