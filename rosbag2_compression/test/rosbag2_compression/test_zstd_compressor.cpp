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

#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "rcpputils/filesystem_helper.hpp"

#include "rcutils/filesystem.h"

#include "rosbag2_compression/zstd_compressor.hpp"
#include "rosbag2_compression/zstd_decompressor.hpp"

#include "rosbag2_test_common/temporary_directory_fixture.hpp"

#include "gmock/gmock.h"

namespace
{
constexpr const char kGarbageStatement[] = "garbage";
constexpr const int kDefaultGarbageFileSize = 10;  // MiB

/**
 * Creates a text file of a certain size.
 * \param uri File path to write file.
 * \param size Size of file in MiB.
 */
void create_garbage_file(const std::string & uri, int size = kDefaultGarbageFileSize)
{
  auto out = std::ofstream{uri};
  out.exceptions(std::ifstream::failbit | std::ifstream::badbit);

  const auto file_size = size * 1024 * 1024;
  const auto num_iterations = file_size / static_cast<int>(strlen(kGarbageStatement));

  for (int i = 0; i < num_iterations; i++) {
    out << kGarbageStatement;
  }
}

std::vector<char> read_file(const std::string & uri)
{
  auto infile = std::ifstream{uri, std::ios_base::binary | std::ios::ate};
  infile.exceptions(std::ifstream::failbit | std::ifstream::badbit);

  const auto file_size = infile.tellg();
  // Initialize contents with size = file_size
  // Uniform initialization cannot be used here since it will choose
  // the initializer list constructor instead.
  auto contents = std::vector<char>(file_size);

  infile.seekg(0, std::ios_base::beg);
  infile.read(contents.data(), file_size);

  return contents;
}
}  // namespace

class CompressionHelperFixture : public rosbag2_test_common::TemporaryDirectoryFixture
{
protected:
  CompressionHelperFixture() = default;

  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }
};

TEST_F(CompressionHelperFixture, zstd_compress_file_uri)
{
  const auto uri = (rcpputils::fs::path(temporary_dir_path_) / "file1.txt").string();
  create_garbage_file(uri);
  auto zstd_compressor = rosbag2_compression::ZstdCompressor{};
  const auto compressed_uri = zstd_compressor.compress_uri(uri);

  const auto expected_compressed_uri = uri + "." + zstd_compressor.get_compression_identifier();
  const auto uncompressed_file_size = rcutils_get_file_size(uri.c_str());
  const auto compressed_file_size = rcutils_get_file_size(compressed_uri.c_str());

  EXPECT_NE(compressed_uri, uri);
  EXPECT_EQ(compressed_uri, expected_compressed_uri);
  EXPECT_LT(compressed_file_size, uncompressed_file_size);
  EXPECT_GT(compressed_file_size, 0u);
  EXPECT_TRUE(rcpputils::fs::exists(compressed_uri)) <<
    "Expected compressed path: \"" << compressed_uri << "\" to exist!";
}

TEST_F(CompressionHelperFixture, zstd_decompress_file_uri)
{
  const auto uri = (rcpputils::fs::path(temporary_dir_path_) / "file1.txt").string();
  create_garbage_file(uri);
  const auto initial_file_size = rcutils_get_file_size(uri.c_str());

  auto zstd_compressor = rosbag2_compression::ZstdCompressor{};
  const auto compressed_uri = zstd_compressor.compress_uri(uri);

// The test is invalid if the initial file is not deleted
  ASSERT_EQ(0, std::remove(uri.c_str())) <<
    "Removal of \"" << uri << "\" failed! The remaining tests require \"" <<
    uri << "\" to be deleted!";

  auto zstd_decompressor = rosbag2_compression::ZstdDecompressor{};
  const auto decompressed_uri = zstd_decompressor.decompress_uri(compressed_uri);

  const auto expected_decompressed_uri = uri;
  const auto decompressed_file_size = rcutils_get_file_size(decompressed_uri.c_str());

  EXPECT_NE(compressed_uri, uri);
  EXPECT_NE(decompressed_uri, compressed_uri);
  EXPECT_EQ(uri, expected_decompressed_uri);
  EXPECT_EQ(initial_file_size, decompressed_file_size);
  EXPECT_TRUE(rcpputils::fs::exists(decompressed_uri)) <<
    "Expected decompressed file: \"" << decompressed_uri << "\" to exist!";
}

TEST_F(CompressionHelperFixture, zstd_decompress_file_contents)
{
  const auto uri = (rcpputils::fs::path(temporary_dir_path_) / "file2.txt").string();
  create_garbage_file(uri);

  ASSERT_TRUE(rcpputils::fs::exists(uri)) <<
    "Expected initial file: \"" << uri << "\" to exist!";

  const auto initial_data = read_file(uri);
  const auto initial_file_size = rcutils_get_file_size(uri.c_str());

  EXPECT_EQ(
    initial_data.size() * sizeof(decltype(initial_data)::value_type),
    initial_file_size) << "Expected data contents of file to equal file size!";

  auto compressor = rosbag2_compression::ZstdCompressor{};
  const auto compressed_uri = compressor.compress_uri(uri);

  ASSERT_TRUE(rcpputils::fs::exists(compressed_uri)) <<
    "Expected compressed file: \"" << compressed_uri << "\" to exist!";

  ASSERT_EQ(0, std::remove(uri.c_str())) <<
    "Removal of initial file: \"" << uri <<
    "\" failed! The remaining tests require \"" << uri << "\" to be deleted.";

  auto decompressor = rosbag2_compression::ZstdDecompressor{};
  const auto decompressed_uri = decompressor.decompress_uri(compressed_uri);

  ASSERT_TRUE(rcpputils::fs::exists(decompressed_uri)) <<
    "Decompressed file: \"" << decompressed_uri << "\" must exist!";

  EXPECT_EQ(uri, decompressed_uri) <<
    "Expected decompressed file name to be same as initial!";

  const auto decompressed_data = read_file(decompressed_uri);
  const auto decompressed_file_size = rcutils_get_file_size(decompressed_uri.c_str());

  EXPECT_EQ(
    decompressed_data.size() * sizeof(decltype(initial_data)::value_type),
    decompressed_file_size) <<
    "Expected data contents of compressed file to equal compressed file size!";

  ASSERT_EQ(initial_file_size, decompressed_file_size) <<
    "Initial file size must be same as decompressed file size!";

  EXPECT_EQ(initial_data, decompressed_data);
}

TEST_F(CompressionHelperFixture, zstd_decompress_fails_on_bad_file)
{
  const auto uri = (rcpputils::fs::path(temporary_dir_path_) / "file3.txt").string();
  create_garbage_file(uri);

  auto decompressor = rosbag2_compression::ZstdDecompressor{};
  EXPECT_THROW(decompressor.decompress_uri(uri), std::runtime_error) <<
    "Expected decompress(\"" << uri << "\") to fail!";
}

TEST_F(CompressionHelperFixture, zstd_decompress_fails_on_bad_uri)
{
  const auto bad_uri = (rcpputils::fs::path(temporary_dir_path_) / "bad_uri.txt").string();
  auto decompressor = rosbag2_compression::ZstdDecompressor{};

  EXPECT_THROW(decompressor.decompress_uri(bad_uri), std::runtime_error) <<
    "Expected decompress_uri(\"" << bad_uri << "\") to fail!";
}
