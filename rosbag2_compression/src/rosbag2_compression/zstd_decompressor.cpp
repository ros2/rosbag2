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
#include <cstdio>
#include <sstream>
#include <string>
#include <vector>

#include "rcpputils/filesystem_helper.hpp"

#include "rosbag2_compression/zstd_decompressor.hpp"

#include "rosbag2_storage/filesystem_helper.hpp"

#include "logging.hpp"

namespace
{
constexpr const char DECOMPRESSION_IDENTIFIER[] = "zstd";

FILE * open_file(const std::string & uri, const std::string & read_mode)
{
  FILE * fp{nullptr};
#ifdef _WIN32
  fopen_s(&fp, uri.c_str(), read_mode.c_str());
#else
  fp = std::fopen(uri.c_str(), read_mode.c_str());
#endif
  return fp;
}

std::vector<uint8_t> get_input_buffer(
  const std::string & uri,
  size_t & compressed_buffer_length)
{
  // Get the file size
  compressed_buffer_length = rosbag2_storage::FilesystemHelper::get_file_size(uri);

  // Read in buffer, handling accordingly
  const auto file_pointer = open_file(uri.c_str(), "rb");
  if (file_pointer == nullptr) {
    std::stringstream errmsg;
    errmsg << "Failed to open file: \"" << uri << "\" for binary reading!";

    throw std::runtime_error(errmsg.str());
  }

  // Allocate and read in
  std::vector<uint8_t> compressed_buffer;
  compressed_buffer.reserve(compressed_buffer_length);

  const auto nRead = fread(
    compressed_buffer.data(), sizeof(uint8_t), compressed_buffer_length, file_pointer);

  if (nRead != compressed_buffer_length) {
    ROSBAG2_COMPRESSION_LOG_ERROR_STREAM("Bytes read !(" <<
      nRead << ") != compressed_buffer_length (" << compressed_buffer_length <<
      ")!");
    // An error indicator is set by fread, so the following check will throw.
  }

  if (ferror(file_pointer)) {
    fclose(file_pointer);

    std::stringstream errmsg;
    errmsg << "Unable to read binary data from file: \"" << uri << "\"!";

    throw std::runtime_error(errmsg.str());
  }

  fclose(file_pointer);
  return compressed_buffer;
}

void write_output_buffer(
  const uint8_t * output_buffer,
  const size_t output_buffer_length,
  const std::string & uri)
{
  const auto file_pointer = open_file(uri.c_str(), "wb");
  if (file_pointer == nullptr) {
    std::stringstream errmsg;
    errmsg << "Failed to open file: \"" << uri << "\" for binary writing!";

    throw std::runtime_error(errmsg.str());
  }

  const auto nWrite = fwrite(
    output_buffer, sizeof(uint8_t), output_buffer_length, file_pointer);

  if (nWrite != output_buffer_length) {
    ROSBAG2_COMPRESSION_LOG_ERROR_STREAM("Bytes written (" <<
      nWrite << " != output_buffer_length (" << output_buffer_length <<
      ")!");
    // An error indicator is set by fwrite, so the following check will throw.
  }

  if (ferror(file_pointer)) {
    fclose(file_pointer);

    std::stringstream errmsg;
    errmsg << "Unable to write decompressed data to file: \"" << uri << "\"!";

    throw std::runtime_error(errmsg.str());
  }

  fclose(file_pointer);
}

void throw_on_zstd_error(const size_t compression_result)
{
  if (ZSTD_isError(compression_result)) {
    std::stringstream error;
    error << "ZSTD decompression error: " << ZSTD_getErrorName(compression_result);

    throw std::runtime_error(error.str());
  }
}

void check_frame_content(const size_t frame_content)
{
  if (frame_content == ZSTD_CONTENTSIZE_ERROR) {
    throw std::runtime_error("File not compressed with Zstd.");
  } else if (frame_content == ZSTD_CONTENTSIZE_UNKNOWN) {
    throw std::runtime_error("Unable to determine file size");
  }
}

void print_decompression_statistics(
  const std::chrono::high_resolution_clock::time_point start,
  const std::chrono::high_resolution_clock::time_point end,
  const size_t decompressed_size,
  const size_t compressed_size)
{
  const auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
  const auto decompression_ratio =
    static_cast<double>(decompressed_size) / static_cast<double>(compressed_size);

  ROSBAG2_COMPRESSION_LOG_DEBUG_STREAM(
    "Decompression statistics:\n" <<
      "Time: " << duration.count() << " microseconds" <<
      "Compression Ratio: " << decompression_ratio);
}
}  // namespace

namespace rosbag2_compression
{

std::string ZstdDecompressor::decompress_uri(const std::string & uri)
{
  const auto start = std::chrono::high_resolution_clock::now();
  const auto uri_path = rcpputils::fs::path(uri);
  const auto decompressed_uri = rcpputils::fs::remove_extension(uri_path).string();

  size_t compressed_buffer_length{0};
  const auto compressed_buffer = get_input_buffer(uri, compressed_buffer_length);

  const auto decompressed_buffer_length =
    ZSTD_getFrameContentSize(compressed_buffer.data(), compressed_buffer_length);
  check_frame_content(decompressed_buffer_length);

  std::vector<uint8_t> decompressed_buffer;
  decompressed_buffer.reserve(decompressed_buffer_length);

  const auto decompression_result = ZSTD_decompress(
    decompressed_buffer.data(), decompressed_buffer_length,
    compressed_buffer.data(), compressed_buffer_length);

  throw_on_zstd_error(decompression_result);

  write_output_buffer(decompressed_buffer.data(), decompressed_buffer_length, decompressed_uri);

  const auto end = std::chrono::high_resolution_clock::now();
  print_decompression_statistics(start, end, decompression_result, compressed_buffer_length);

  return decompressed_uri;
}

void ZstdDecompressor::decompress_serialized_bag_message(
  rosbag2_storage::SerializedBagMessage *)
{
  throw std::logic_error("Not implemented");
}

std::string ZstdDecompressor::get_decompression_identifier() const
{
  return DECOMPRESSION_IDENTIFIER;
}
}  // namespace rosbag2_compression
