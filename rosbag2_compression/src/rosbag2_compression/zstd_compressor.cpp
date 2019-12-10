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

#include "logging.hpp"
#include "rosbag2_compression/zstd_compressor.hpp"

namespace
{
// Increasing the compression level will:
//   - Increase the time taken to compress
//   - Decrease the size of the compressed data
// Setting to zero uses Zstd's default value of 3.
constexpr const int DEFAULT_ZSTD_COMPRESSION_LEVEL = 1;
}

namespace rosbag2_compression
{

std::unique_ptr<char[]> ZstdCompressor::get_input_buffer(const std::string & uri)
{
  try {
    std::ifstream infile{uri};
    infile.exceptions(std::ifstream::failbit);
    // Get size and allocate
    infile.seekg(0, std::ios::end);
    const size_t decompressed_buffer_length = infile.tellg();
    auto decompressed_buffer = std::make_unique<char[]>(decompressed_buffer_length);
    // Go back and read in contents
    infile.seekg(0 /* off */, std::ios::beg);
    infile.read(decompressed_buffer.get(), decompressed_buffer_length);
    return decompressed_buffer;
  } catch (std::ios_base::failure & fail) {
    std::cerr << "Caught IO stream exception while reading file: " << fail.what() << std::endl;
    throw fail;
  }
}

void ZstdCompressor::check_compression_result(const size_t & compression_result)
{
  if (ZSTD_isError(compression_result)) {
    std::stringstream error;
    error << "ZSTD compression error: " << ZSTD_getErrorName(compression_result);
    throw std::runtime_error(error.str());
  }
  ROSBAG2_COMPRESSION_LOG_DEBUG("ZSTD compressed file.");
}

void ZstdCompressor::print_compression_statistics(
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

std::string ZstdCompressor::compress_uri(const std::string & uri)
{
  const auto start = std::chrono::high_resolution_clock::now();
  const auto compressed_uri = uri + "." + get_compression_identifier();
  try {
    auto decompressed_buffer = get_input_buffer(uri);
    auto decompressed_buffer_length = strlen(decompressed_buffer.get());
    // Allocate based on compression bound and compress
    const size_t compressed_buffer_length = ZSTD_compressBound(decompressed_buffer_length);
    auto compressed_buffer = std::make_unique<char[]>(compressed_buffer_length);
    // Perform compression and check
    const size_t compression_result = ZSTD_compress(
      compressed_buffer.get(), compressed_buffer_length,
      decompressed_buffer.get(), decompressed_buffer_length, DEFAULT_ZSTD_COMPRESSION_LEVEL);
    check_compression_result(compression_result);
    std::ofstream outfile{compressed_uri, std::ios::out | std::ios::binary};
    outfile.write(compressed_buffer.get(), compression_result);
    const auto end = std::chrono::high_resolution_clock::now();
    print_compression_statistics(start, end, decompressed_buffer_length, compression_result);
  } catch (std::ios_base::failure & fail) {
    std::cerr << "Caught IO stream exception while compressing: " << fail.what() << std::endl;
    throw fail;
  }
  return compressed_uri;
}

void ZstdCompressor::compress_serialized_bag_message(
  rosbag2_storage::SerializedBagMessage *)
{
  throw std::logic_error("Not implemented");
}

std::string ZstdCompressor::get_compression_identifier() const
{
  static std::string kCompressionIdentifier = "zstd";
  return kCompressionIdentifier;
}

}  // namespace rosbag2_compression
