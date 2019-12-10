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

#include <chrono>
#include <memory>
#include <string>

#include "rcpputils/filesystem_helper.hpp"

#include "logging.hpp"
#include "rosbag2_compression/zstd_decompressor.hpp"

namespace
{
std::unique_ptr<char[]> get_input_buffer(const std::string & uri, size_t & buffer_length)
{
  try {
    std::ifstream infile{uri};
    infile.exceptions(std::ifstream::failbit);
    // Get size and allocate
    infile.seekg(0, std::ios::end);
    buffer_length = infile.tellg();
    auto decompressed_buffer = std::make_unique<char[]>(buffer_length);
    // Go back and read in contents
    infile.seekg(0 /* off */, std::ios::beg);
    infile.read(decompressed_buffer.get(), buffer_length);
    return decompressed_buffer;
  } catch (std::ios_base::failure & fail) {
    ROSBAG2_COMPRESSION_LOG_ERROR_STREAM(
      "Caught IO stream exception while reading file: " << fail.what());
    throw fail;
  }
}

void check_frame_content(const size_t frame_content)
{
  if (frame_content == ZSTD_CONTENTSIZE_ERROR) {
    throw std::runtime_error("File not compressed with Zstd.");
  }
  if (frame_content == ZSTD_CONTENTSIZE_UNKNOWN) {
    ROSBAG2_COMPRESSION_LOG_WARN("Unable to determine file size, considering upper bound.");
  }
}

void check_decompression_result(const size_t decompression_result)
{
  if (ZSTD_isError(decompression_result)) {
    std::stringstream error;
    error << "ZSTD compression error: " << ZSTD_getErrorName(decompression_result);
    throw std::runtime_error(error.str());
  }
  ROSBAG2_COMPRESSION_LOG_DEBUG("ZSTD decompressed file.");
}

void print_statistics(
  std::chrono::high_resolution_clock::time_point start,
  std::chrono::high_resolution_clock::time_point end)
{
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
  ROSBAG2_COMPRESSION_LOG_DEBUG_STREAM(
    "Decompression statistics:\n" <<
      "Time: " << duration.count() << " microseconds");
}
}  // namespace

namespace rosbag2_compression
{

std::string ZstdDecompressor::decompress_uri(const std::string & uri)
{
  auto start = std::chrono::high_resolution_clock::now();
  try {
    // Prepare the compression buffers
    size_t compressed_buffer_length{0};
    auto compressed_buffer = get_input_buffer(uri, compressed_buffer_length);
    const size_t decompressed_buffer_length =
      ZSTD_getFrameContentSize(compressed_buffer.get(), compressed_buffer_length);
    check_frame_content(decompressed_buffer_length);
    auto decompressed_buffer = std::make_unique<char[]>(decompressed_buffer_length);
    // Perform decompression
    const size_t decompression_result = ZSTD_decompress(
      decompressed_buffer.get(), decompressed_buffer_length,
      compressed_buffer.get(), compressed_buffer_length);
    check_decompression_result(decompression_result);
    // Remove extension and write decompressed file
    auto uri_path = rcpputils::fs::path(uri);
    auto decompressed_uri = rcpputils::fs::remove_extension(uri_path);
    std::ofstream outfile(decompressed_uri.string());
    outfile.exceptions(std::ofstream::failbit);
    outfile.write(decompressed_buffer.get(), decompression_result);
    outfile.close();
    auto end = std::chrono::high_resolution_clock::now();
    print_statistics(start, end);
    return decompressed_uri.string();
  } catch (std::ios_base::failure & fail) {
    ROSBAG2_COMPRESSION_LOG_ERROR_STREAM(
      "Caught IO stream exception while decompressing: " << fail.what());
    throw fail;
  }
}

void ZstdDecompressor::decompress_serialized_bag_message(
  rosbag2_storage::SerializedBagMessage *)
{
  throw std::logic_error("Not implemented");
}

std::string ZstdDecompressor::get_decompression_identifier() const
{
  static std::string kCompressionIdentifier = "zstd";
  return kCompressionIdentifier;
}
}  // namespace rosbag2_compression
