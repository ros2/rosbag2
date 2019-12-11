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

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rosbag2_compression/zstd_compressor.hpp"

#include "rosbag2_storage/filesystem_helper.hpp"

#include "logging.hpp"

namespace
{
// Increasing the compression level will:
//   - Increase the time taken to compress
//   - Decrease the size of the compressed data
// Setting to zero uses Zstd's default value of 3.
constexpr const int DEFAULT_ZSTD_COMPRESSION_LEVEL = 1;

constexpr const char COMPRESSION_IDENTIFIER[] = "zstd";

std::vector<uint8_t> get_input_buffer(const std::string & uri)
{
  // Get the file size
  const auto decompressed_buffer_length = rosbag2_storage::FilesystemHelper::get_file_size(uri);
  // Read in buffer, handling accordingly
  auto file_pointer = std::fopen(uri.c_str(), "rb");
  if (file_pointer == nullptr) {
    fclose(file_pointer);
    throw std::runtime_error("Error opening file");
  }
  // Allocate and read in
  std::vector<uint8_t> decompressed_buffer;
  decompressed_buffer.reserve(decompressed_buffer_length);
  fread(decompressed_buffer.data(), sizeof(uint8_t), decompressed_buffer_length, file_pointer);
  if (ferror(file_pointer)) {
    fclose(file_pointer);
    throw std::runtime_error("Unable to read file");
  }
  fclose(file_pointer);
  return decompressed_buffer;
}

void write_output_buffer(
  const std::vector<uint8_t> & output_buffer,
  const std::string & uri)
{
  auto file_pointer = std::fopen(uri.c_str(), "wb");
  fwrite(output_buffer.data(), sizeof(uint8_t), output_buffer.capacity(), file_pointer);
  if (ferror(file_pointer)) {
    fclose(file_pointer);
    throw std::runtime_error("Unable to write compressed file");
  }
  fclose(file_pointer);
}

void throw_on_zstd_error(const size_t compression_result)
{
  if (ZSTD_isError(compression_result)) {
    std::stringstream error;
    error << "ZSTD compression error: " << ZSTD_getErrorName(compression_result);
    throw std::runtime_error(error.str());
  }
  ROSBAG2_COMPRESSION_LOG_DEBUG("ZSTD compressed file.");
}

void print_compression_statistics(
  std::chrono::high_resolution_clock::time_point start,
  std::chrono::high_resolution_clock::time_point end,
  size_t decompressed_size, size_t compressed_size)
{
  const auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
  const auto compression_ratio =
    static_cast<double>(decompressed_size) / static_cast<double>(compressed_size);
  ROSBAG2_COMPRESSION_LOG_DEBUG_STREAM(
    "Compression statistics:\n" <<
      "Time: " << duration.count() << " microseconds" <<
      "Compression Ratio: " << compression_ratio
  );
}
}  // namespace

namespace rosbag2_compression
{

std::string ZstdCompressor::compress_uri(const std::string & uri)
{
  const auto start = std::chrono::high_resolution_clock::now();
  const auto compressed_uri = uri + "." + get_compression_identifier();
  const auto decompressed_buffer = get_input_buffer(uri);
  const auto decompressed_buffer_length = decompressed_buffer.capacity();
  // Allocate based on compression bound and compress
  const size_t compressed_buffer_length = ZSTD_compressBound(decompressed_buffer_length);
  std::vector<uint8_t> compressed_buffer;
  compressed_buffer.reserve(compressed_buffer_length);
  // Perform compression and check
  const size_t compression_result = ZSTD_compress(
    compressed_buffer.data(), compressed_buffer_length,
    decompressed_buffer.data(), decompressed_buffer_length, DEFAULT_ZSTD_COMPRESSION_LEVEL);
  throw_on_zstd_error(compression_result);
  write_output_buffer(compressed_buffer, uri);
  const auto end = std::chrono::high_resolution_clock::now();
  print_compression_statistics(start, end, decompressed_buffer_length, compression_result);
  return compressed_uri;
}

void ZstdCompressor::compress_serialized_bag_message(
  rosbag2_storage::SerializedBagMessage *)
{
  throw std::logic_error("Not implemented");
}

std::string ZstdCompressor::get_compression_identifier() const
{
  return COMPRESSION_IDENTIFIER;
}

}  // namespace rosbag2_compression
