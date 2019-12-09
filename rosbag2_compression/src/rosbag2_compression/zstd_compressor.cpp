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

#include "logging.hpp"
#include "rosbag2_compression/zstd_compressor.hpp"

namespace rosbag2_compression
{

std::string ZstdCompressor::compress_uri(const std::string & uri)
{
  auto start = std::chrono::high_resolution_clock::now();
  auto compressed_uri = uri + "." + get_compression_identifier();
  {
    std::ifstream infile{uri};
    if (infile) {
      // Get size and allocate
      infile.seekg(0, std::ios::end);
      size_t decompressed_buffer_length = infile.tellg();
      char * decompressed_buffer = new char[decompressed_buffer_length];
      // Go back and read in contents
      infile.seekg(0, std::ios::beg);
      infile.read(decompressed_buffer, decompressed_buffer_length);
      // Allocate based on compression bound and compress
      size_t const compressed_buffer_length = ZSTD_compressBound(decompressed_buffer_length);
      char* compressed_buffer = new char[compressed_buffer_length];
      size_t const compression_result = ZSTD_compress(
        compressed_buffer, compressed_buffer_length,
        decompressed_buffer, decompressed_buffer_length, 1);
      if(ZSTD_isError(compression_result)){
        std::stringstream error;
        error << "ZSTD compression error: " << ZSTD_getErrorName(compression_result);
        throw std::runtime_error(error.str());
      }
    }
    std::ofstream outfile(compressed_uri, std::ios::out | std::ios::binary);
    if (!outfile) {
      std::stringstream error;
      error << "Unable to write " << compressed_uri << " to file.";
      throw std::runtime_error(error.str());
    }
    outfile.write(compressed_buffer, compression_result);
    // Statistics
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    auto compression_ratio = 
      static_cast<float>(decompressed_buffer_length) / static_cast<float>(compression_result);
    ROSBAG2_COMPRESSION_LOG_INFO("ZSTD compressed file.");
    ROSBAG2_COMPRESSION_LOG_DEBUG("Compression statistics:");
    ROSBAG2_COMPRESSION_LOG_DEBUG_STREAM("Time: " << duration.count() << " microseconds");
    ROSBAG2_COMPRESSION_LOG_DEBUG_STREAM("Compression Ratio: " << compression_ratio);
    delete[] compressed_buffer;
    delete[] decompressed_buffer;
    return compressed_uri;
  }
  std::stringstream error;
  error << "Unable to read " << uri.c_str();
  throw std::runtime_error(error.str());
}

void ZstdCompressor::compress_serialized_bag_message(
  rosbag2_storage::SerializedBagMessage * bag_message)
{
  throw std::logic_error("Not implemented");
}

std::string ZstdCompressor::get_compression_identifier() const
{
  return "zstd";
}

}  // namespace rosbag2_compression
