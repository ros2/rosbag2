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

#include <cstdio>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <random>
#include <sstream>
#include <string>

using namespace ::testing;  // NOLINT

namespace rosbag2_test_common
{

/// \brief Construct a uniquely named temporary directory, in "parent", with format base_nameXXXXXX
/// The output, if successful, is guaranteed to be a newly-created directory.
/// The underlying implementation keeps generating paths until one that does not exist is found or
/// until the number of iterations exceeded the maximum tries.
/// This guarantees that there will be no existing files in the returned directory.
/// \param[in] base_name User-specified portion of the created directory.
/// \param[in] parent_path The parent path of the directory that will be created.
/// \param[in] max_tries The maximum number of tries to find a unique directory (default 1000)
/// \return A path to a newly-created directory with base_name and a 6-character unique suffix.
/// \throws std::system_error If any OS APIs do not succeed.
/// \throws std::runtime_error If the number of the iterations exceeds the maximum tries and
/// a unique directory is not found.
std::filesystem::path create_temp_directory(
  const std::string & base_name,
  std::filesystem::path parent_path = std::filesystem::temp_directory_path(),
  size_t max_tries = 1000)
{
  // mersenne twister random generator engine seeded with the std::random_device
  std::mt19937 random_generator(std::random_device{}());
  std::uniform_int_distribution<> distribution(0, 999999);
  std::filesystem::path path_to_temp_dir;
  constexpr size_t kSuffixLength = 7;  // 6 chars + 1 null terminator
  char random_suffix_str[kSuffixLength];
  size_t current_iteration = 0;
  while (true) {
    snprintf(random_suffix_str, kSuffixLength, "%06x", distribution(random_generator));
    const std::string random_dir_name = base_name + random_suffix_str;
    path_to_temp_dir = parent_path / random_dir_name;
    // true if the directory was newly created.
    if (std::filesystem::create_directory(path_to_temp_dir)) {
      break;
    }
    if (current_iteration == max_tries) {
      throw std::runtime_error(
              "Exceeded maximum allowed iterations to find non-existing directory");
    }
    current_iteration++;
  }
  return path_to_temp_dir;
}

class TemporaryDirectoryFixture : public Test
{
public:
  TemporaryDirectoryFixture()
  {
    temporary_dir_path_ = create_temp_directory("tmp_test_dir_").string();
  }

  ~TemporaryDirectoryFixture() override
  {
    std::filesystem::remove_all(std::filesystem::path(temporary_dir_path_));
  }

  std::string temporary_dir_path_;
};

/**
 * @brief parametrizes the temporary directory fixture above with a string parameter,
 * which can be used for testing across several storage plugins.
 */
class ParametrizedTemporaryDirectoryFixture
  : public TemporaryDirectoryFixture,
  public WithParamInterface<std::string> {};

}  // namespace rosbag2_test_common

#endif  // ROSBAG2_TEST_COMMON__TEMPORARY_DIRECTORY_FIXTURE_HPP_
