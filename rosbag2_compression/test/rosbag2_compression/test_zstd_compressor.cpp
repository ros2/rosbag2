// Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#include <fstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rosbag2_compression/zstd_compressor.hpp"
#include "rosbag2_storage/filesystem_helper.hpp"
#include "rosbag2_test_common/temporary_directory_fixture.hpp"

#include "gmock/gmock.h"

class FilesystemHelperFixture : public rosbag2_test_common::TemporaryDirectoryFixture {};
constexpr const char * GARBAGE_STATEMENT = "garbage";
constexpr const int DEFAULT_GARBAGE_FILE_SIZE = 10 * 1024 * 1024;  // 10 MB
constexpr const int DEFAULT_ITERATIONS = DEFAULT_GARBAGE_FILE_SIZE / sizeof(GARBAGE_STATEMENT);

/**
 * Creates a 10 MiB file.
 * \param uri File path to write file.
 * \param num_iterations Number of times to write garbage statement.
 */
void create_garbage_file(const std::string & uri, int num_iterations = DEFAULT_ITERATIONS)
{
  std::ofstream out(uri);
  if (out) {
    for (int i = 0; i < num_iterations; i++) {
      out << GARBAGE_STATEMENT;
    }
    return;
  }
  throw std::runtime_error("Unable to write to file.");
}

TEST_F(FilesystemHelperFixture, zstd_compress_file_uri)
{
  // For rcl logging macros
  rclcpp::init(0, nullptr);
  const auto uri = rosbag2_storage::FilesystemHelper::concat({temporary_dir_path_, "file1.txt"});
  create_garbage_file(uri);
  auto zstd_compressor = rosbag2_compression::ZstdCompressor();
  auto compressed_uri = zstd_compressor.compress_uri(uri);

  const auto expected_compressed_uri = uri + "." + zstd_compressor.get_compression_identifier();
  const auto uncompressed_file_size = rosbag2_storage::FilesystemHelper::get_file_size(uri);
  const auto compressed_file_size =
    rosbag2_storage::FilesystemHelper::get_file_size(compressed_uri);
  rclcpp::shutdown();

  EXPECT_EQ(compressed_uri, expected_compressed_uri);
  EXPECT_LT(compressed_file_size, uncompressed_file_size);
}
