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
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "rcpputils/filesystem_helper.hpp"

#include "rosbag2_compression/zstd_compressor.hpp"
#include "rosbag2_compression/zstd_decompressor.hpp"

#include "rosbag2_test_common/temporary_directory_fixture.hpp"

#include "gmock/gmock.h"

namespace
{
constexpr const char kGarbageStatement[] = "garbage";
constexpr const int kDefaultGarbageFileSize = 10;  // MiB

/**
 * Writes 1M * size garbage data to a stream.
 * @param out
 * @param size
 */
void write_garbage_stream(std::ostream & out, int size = kDefaultGarbageFileSize)
{
  const auto output_size = size * 1024 * 1024;
  const auto num_iterations = output_size / static_cast<int>(strlen(kGarbageStatement));

  for (int i = 0; i < num_iterations; i++) {
    out << kGarbageStatement;
  }
}

/**
 * Creates a text file of a certain size.
 * \param uri File path to write file.
 * \param size Size of file in MiB.
 */
void create_garbage_file(const std::string & uri, int size = kDefaultGarbageFileSize)
{
  auto out = std::ofstream{uri};
  out.exceptions(std::ifstream::failbit | std::ifstream::badbit);

  write_garbage_stream(out, size);
}

std::string create_garbage_string(int size = kDefaultGarbageFileSize)
{
  std::stringstream output;
  write_garbage_stream(output, size);
  return output.str();
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
    allocator_ = rcutils_get_default_allocator();
    message_ = create_garbage_string();

    rclcpp::init(0, nullptr);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }

  int write_data_to_serialized_string_message(
    uint8_t * buffer, size_t buffer_capacity, const std::string & message)
  {
    // This function also writes the final null charachter, which is absent in the CDR format.
    // Here this behaviour is ok, because we only test test writing and reading from/to sqlite.
    return rcutils_snprintf(
      reinterpret_cast<char *>(buffer),
      buffer_capacity,
      "%c%c%c%c%c%c%c%c%s",
      0x00, 0x01, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00,
      message.c_str());
  }

  int get_buffer_capacity(const std::string & message)
  {
    return write_data_to_serialized_string_message(nullptr, 0, message);
  }

  std::shared_ptr<rcutils_uint8_array_t> make_serialized_message(std::string message)
  {
    int message_size = get_buffer_capacity(message);
    message_size++;  // need to account for terminating null character
    assert(message_size > 0);

    auto msg = new rcutils_uint8_array_t;
    *msg = rcutils_get_zero_initialized_uint8_array();
    auto ret = rcutils_uint8_array_init(msg, message_size, &allocator_);
    if (ret != RCUTILS_RET_OK) {
      throw std::runtime_error("Error allocating resources " + std::to_string(ret));
    }

    auto serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
      msg,
      [](rcutils_uint8_array_t * msg) {
        int error = rcutils_uint8_array_fini(msg);
        delete msg;
        if (error != RCUTILS_RET_OK) {
          RCUTILS_LOG_ERROR_NAMED(
            "rosbag2_compression", "Leaking memory %i", error);
        }
      });

    serialized_data->buffer_length = message_size;
    int written_size = write_data_to_serialized_string_message(
      serialized_data->buffer, serialized_data->buffer_capacity, message);

    assert(written_size == message_size - 1);  // terminated null character not counted
    (void) written_size;
    return serialized_data;
  }

  std::string deserialize_message(std::shared_ptr<rcutils_uint8_array_t> serialized_message)
  {
    uint8_t * copied = new uint8_t[serialized_message->buffer_length];
    auto string_length = serialized_message->buffer_length - 8;
    memcpy(copied, &serialized_message->buffer[8], string_length);
    std::string message_content(reinterpret_cast<char *>(copied));
    // cppcheck-suppress mismatchAllocDealloc ; complains about "copied" but used new[] and delete[]
    delete[] copied;
    return message_content;
  }

  rcutils_allocator_t allocator_;
  std::string message_;
  size_t compressed_length_{991};  // manually calculated, could change if compression params change
};

TEST_F(CompressionHelperFixture, zstd_compress_file_uri)
{
  const auto uri = (rcpputils::fs::path(temporary_dir_path_) / "file1.txt").string();
  create_garbage_file(uri);

  ASSERT_TRUE(rcpputils::fs::exists(uri)) <<
    "Expected uncompressed URI: \"" << uri << "\" to exist.";

  auto zstd_compressor = rosbag2_compression::ZstdCompressor{};
  const auto compressed_uri = zstd_compressor.compress_uri(uri);

  ASSERT_TRUE(rcpputils::fs::exists(compressed_uri)) <<
    "Expected compressed URI: \"" << compressed_uri << "\" to exist.";

  const auto expected_compressed_uri = uri + "." + zstd_compressor.get_compression_identifier();
  const auto uncompressed_file_size = rcpputils::fs::file_size(rcpputils::fs::path{uri});
  const auto compressed_file_size = rcpputils::fs::file_size(rcpputils::fs::path{compressed_uri});

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

  const auto initial_file_path = rcpputils::fs::path{uri};

  ASSERT_TRUE(initial_file_path.exists()) <<
    "Expected initial file: \"" << initial_file_path.string() <<
    "\" to exist.";

  const auto initial_file_size = initial_file_path.file_size();

  auto zstd_compressor = rosbag2_compression::ZstdCompressor{};
  const auto compressed_uri = zstd_compressor.compress_uri(uri);

  // The test is invalid if the initial file is not deleted
  ASSERT_TRUE(rcpputils::fs::remove(initial_file_path)) <<
    "Removal of \"" << initial_file_path.string() <<
    "\" failed! The remaining tests require \"" <<
    initial_file_path.string() << "\" to be deleted!";

  auto zstd_decompressor = rosbag2_compression::ZstdDecompressor{};
  const auto decompressed_uri = zstd_decompressor.decompress_uri(compressed_uri);
  const auto decompressed_file_path = rcpputils::fs::path{decompressed_uri};
  const auto expected_decompressed_uri = uri;

  ASSERT_TRUE(decompressed_file_path.exists()) <<
    "Expected decompressed file: \"" << decompressed_file_path.string() <<
    "\" to exist.";

  const auto decompressed_file_size = decompressed_file_path.file_size();

  EXPECT_NE(compressed_uri, uri);
  EXPECT_NE(decompressed_uri, compressed_uri);
  EXPECT_EQ(uri, expected_decompressed_uri);
  EXPECT_EQ(initial_file_size, decompressed_file_size);
}

TEST_F(CompressionHelperFixture, zstd_decompress_file_contents)
{
  const auto uri = (rcpputils::fs::path(temporary_dir_path_) / "file2.txt").string();
  create_garbage_file(uri);

  const auto initial_file_path = rcpputils::fs::path{uri};
  ASSERT_TRUE(initial_file_path.exists()) <<
    "Expected initial file: \"" << uri << "\" to exist!";

  const auto initial_data = read_file(uri);
  const auto initial_file_size = initial_file_path.file_size();

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
  const auto decompressed_file_path = rcpputils::fs::path{decompressed_uri};

  ASSERT_TRUE(decompressed_file_path.exists()) <<
    "Decompressed file: \"" << decompressed_file_path.string() << "\" must exist!";

  EXPECT_EQ(uri, decompressed_uri) <<
    "Expected decompressed file name to be same as initial!";

  const auto decompressed_data = read_file(decompressed_uri);
  const auto decompressed_file_size = decompressed_file_path.file_size();

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

TEST_F(CompressionHelperFixture, zstd_compress_serialized_bag_message)
{
  auto msg = std::make_unique<rosbag2_storage::SerializedBagMessage>();
  msg->serialized_data.reset(new rcutils_uint8_array_t);
  msg->serialized_data = make_serialized_message(message_);

  rosbag2_compression::ZstdCompressor compressor;
  compressor.compress_serialized_bag_message(msg.get());

  ASSERT_EQ(compressed_length_, msg->serialized_data->buffer_length);
}

TEST_F(CompressionHelperFixture, zstd_decompress_serialized_bag_message)
{
  auto msg = std::make_unique<rosbag2_storage::SerializedBagMessage>();
  msg->serialized_data.reset(new rcutils_uint8_array_t);
  msg->serialized_data = make_serialized_message(message_);

  rosbag2_compression::ZstdCompressor compressor;
  compressor.compress_serialized_bag_message(msg.get());

  const auto compressed_length = msg->serialized_data->buffer_length;
  EXPECT_EQ(compressed_length_, compressed_length);

  rosbag2_compression::ZstdDecompressor decompressor;
  EXPECT_NO_THROW(decompressor.decompress_serialized_bag_message(msg.get()));
  std::string new_msg = deserialize_message(msg->serialized_data);
  EXPECT_EQ(new_msg, message_);
}
