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

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "rcpputils/filesystem_helper.hpp"

#include "compression_utils.hpp"
#include "rosbag2_compression/zstd_compressor.hpp"

namespace rosbag2_compression
{
ZstdCompressor::ZstdCompressor()
{
  zstd_context_ = ZSTD_createCCtx();
}

ZstdCompressor::~ZstdCompressor()
{
  ZSTD_freeCCtx(zstd_context_);
}

std::string ZstdCompressor::compress_uri(const std::string & uri)
{
  const auto start = std::chrono::high_resolution_clock::now();
  const auto compressed_uri = uri + "." + get_compression_identifier();
  const auto decompressed_buffer = get_input_buffer(uri);

  // Allocate based on compression bound and compress
  const auto compressed_buffer_length = ZSTD_compressBound(decompressed_buffer.size());
  std::vector<uint8_t> compressed_buffer(compressed_buffer_length);

  // Perform compression and check.
  // compression_result is either the actual compressed size or an error code.
  const auto compression_result = ZSTD_compressCCtx(
    zstd_context_,
    compressed_buffer.data(), compressed_buffer.size(),
    decompressed_buffer.data(), decompressed_buffer.size(), kDefaultZstdCompressionLevel);
  throw_on_zstd_error(compression_result);

  // Compression_buffer_length might be larger than the actual compression size
  // Resize compressed_buffer so its size is the actual compression size.
  compressed_buffer.resize(compression_result);
  write_output_buffer(compressed_buffer, compressed_uri);

  const auto end = std::chrono::high_resolution_clock::now();
  print_compression_statistics(start, end, decompressed_buffer.size(), compression_result);
  return compressed_uri;
}

void ZstdCompressor::compress_serialized_bag_message(
  rosbag2_storage::SerializedBagMessage * message)
{
  const auto start = std::chrono::high_resolution_clock::now();
  // Allocate based on compression bound and compress
  const auto uncompressed_buffer_length =
    ZSTD_compressBound(message->serialized_data->buffer_length);
  std::vector<uint8_t> compressed_buffer(uncompressed_buffer_length);

  // Perform compression and check.
  // compression_result is either the actual compressed size or an error code.
  const auto compression_result = ZSTD_compressCCtx(
    zstd_context_,
    compressed_buffer.data(), compressed_buffer.size(),
    message->serialized_data->buffer, message->serialized_data->buffer_length,
    kDefaultZstdCompressionLevel);
  throw_on_zstd_error(compression_result);

  // Compression_buffer_length might be larger than the actual compression size
  // Resize compressed_buffer so its size is the actual compression size.
  compressed_buffer.resize(compression_result);

  const auto resize_result =
    rcutils_uint8_array_resize(message->serialized_data.get(), compression_result);
  throw_on_rcutils_resize_error(resize_result);

  // Note that rcutils_uint8_array_resize changes buffer_capacity but not buffer_length, we
  // have to do that manually.
  message->serialized_data->buffer_length = compression_result;
  std::copy(compressed_buffer.begin(), compressed_buffer.end(), message->serialized_data->buffer);

  const auto end = std::chrono::high_resolution_clock::now();
  print_compression_statistics(start, end, uncompressed_buffer_length, compression_result);
}

std::string ZstdCompressor::get_compression_identifier() const
{
  return kCompressionIdentifier;
}

}  // namespace rosbag2_compression
