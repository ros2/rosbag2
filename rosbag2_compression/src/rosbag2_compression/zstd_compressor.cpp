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

std::string ZstdCompressor::compress_uri(const std::string & uri)
{
  const auto start = std::chrono::high_resolution_clock::now();
  const auto compressed_uri = uri + "." + get_compression_identifier();
  std::ifstream infile{uri};
  if (!infile) {
    std::stringstream error;
    error << "Unable to read " << uri.c_str();
    throw std::runtime_error(error.str());
  }
  // Get size and allocate
  infile.seekg(0, std::ios::end);
  const size_t decompressed_buffer_length = infile.tellg();
  auto decompressed_buffer = std::make_unique<char[]>(decompressed_buffer_length);
  // Go back and read in contents
  infile.seekg(0, std::ios::beg);
  infile.read(decompressed_buffer.get(), decompressed_buffer_length);
  // Allocate based on compression bound and compress
  const size_t compressed_buffer_length = ZSTD_compressBound(decompressed_buffer_length);
  auto compressed_buffer = std::make_unique<char[]>(compressed_buffer_length);
  const size_t compression_result = ZSTD_compress(
    compressed_buffer.get(), compressed_buffer_length,
    decompressed_buffer.get(), decompressed_buffer_length, DEFAULT_ZSTD_COMPRESSION_LEVEL);
  if (ZSTD_isError(compression_result)) {
    std::stringstream error;
    error << "ZSTD compression error: " << ZSTD_getErrorName(compression_result);
    throw std::runtime_error(error.str());
  }
  std::ofstream outfile{compressed_uri, std::ios::out | std::ios::binary};
  if (!outfile) {
    std::stringstream error;
    error << "Unable to write " << compressed_uri << " to file.";
    throw std::runtime_error(error.str());
  }
  outfile.write(compressed_buffer.get(), compression_result);
  // Statistics
  const auto end = std::chrono::high_resolution_clock::now();
  const auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
  const auto compression_ratio =
    static_cast<double>(decompressed_buffer_length) / static_cast<double>(compression_result);
  ROSBAG2_COMPRESSION_LOG_INFO("ZSTD compressed file.");
  ROSBAG2_COMPRESSION_LOG_DEBUG("Compression statistics:");
  ROSBAG2_COMPRESSION_LOG_DEBUG_STREAM("Time: " << duration.count() << " microseconds");
  ROSBAG2_COMPRESSION_LOG_DEBUG_STREAM("Compression Ratio: " << compression_ratio);
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
