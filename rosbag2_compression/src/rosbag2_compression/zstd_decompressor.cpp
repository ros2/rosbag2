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
#include <sstream>
#include <string>
#include <vector>

#include "rcpputils/filesystem_helper.hpp"

#include "compression_utils.hpp"
#include "rosbag2_compression/zstd_decompressor.hpp"

namespace rosbag2_compression
{
ZstdDecompressor::ZstdDecompressor()
{
  zstd_context_ = ZSTD_createDCtx();
}

ZstdDecompressor::~ZstdDecompressor()
{
  ZSTD_freeDCtx(zstd_context_);
}

std::string ZstdDecompressor::decompress_uri(const std::string & uri)
{
  const auto start = std::chrono::high_resolution_clock::now();
  const auto uri_path = rcpputils::fs::path{uri};
  const auto decompressed_uri = rcpputils::fs::remove_extension(uri_path).string();
  const auto compressed_buffer = get_input_buffer(uri);
  const auto compressed_buffer_length = compressed_buffer.size();

  const auto decompressed_buffer_length =
    ZSTD_getFrameContentSize(compressed_buffer.data(), compressed_buffer_length);

  throw_on_invalid_frame_content(decompressed_buffer_length);

  // Initializes decompressed_buffer with size = decompressed_buffer_length.
  // Uniform initialization cannot be used here since it will choose
  // the initializer list constructor instead.
  std::vector<uint8_t> decompressed_buffer(decompressed_buffer_length);

  const auto decompression_result = ZSTD_decompressDCtx(
    zstd_context_,
    decompressed_buffer.data(), decompressed_buffer_length,
    compressed_buffer.data(), compressed_buffer_length);

  throw_on_zstd_error(decompression_result);
  decompressed_buffer.resize(decompression_result);

  write_output_buffer(decompressed_buffer, decompressed_uri);

  const auto end = std::chrono::high_resolution_clock::now();
  print_compression_statistics(start, end, decompression_result, compressed_buffer_length);

  return decompressed_uri;
}

void ZstdDecompressor::decompress_serialized_bag_message(
  rosbag2_storage::SerializedBagMessage * message)
{
  const auto start = std::chrono::high_resolution_clock::now();
  const auto compressed_buffer_length = message->serialized_data->buffer_length;

  const auto decompressed_buffer_length =
    ZSTD_getFrameContentSize(message->serialized_data->buffer, compressed_buffer_length);

  throw_on_invalid_frame_content(decompressed_buffer_length);

  // Initializes decompressed_buffer with size = decompressed_buffer_length.
  // Uniform initialization cannot be used here since it will choose
  // the initializer list constructor instead.
  std::vector<uint8_t> decompressed_buffer(decompressed_buffer_length);

  const auto decompression_result = ZSTD_decompressDCtx(
    zstd_context_,
    decompressed_buffer.data(), decompressed_buffer_length,
    message->serialized_data->buffer, compressed_buffer_length);

  throw_on_zstd_error(decompression_result);

  const auto resize_result =
    rcutils_uint8_array_resize(message->serialized_data.get(), decompression_result);
  throw_on_rcutils_resize_error(resize_result);

  message->serialized_data->buffer_length = decompression_result;
  std::copy(
    decompressed_buffer.begin(), decompressed_buffer.end(),
    message->serialized_data->buffer);

  const auto end = std::chrono::high_resolution_clock::now();
  print_compression_statistics(start, end, decompression_result, compressed_buffer_length);
}

std::string ZstdDecompressor::get_decompression_identifier() const
{
  return kDecompressionIdentifier;
}
}  // namespace rosbag2_compression
