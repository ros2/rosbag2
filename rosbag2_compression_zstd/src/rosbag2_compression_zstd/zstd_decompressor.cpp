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
#include "rosbag2_compression_zstd/zstd_decompressor.hpp"

namespace rosbag2_compression_zstd
{
ZstdDecompressor::ZstdDecompressor()
{
  // From the zstd manual: https://facebook.github.io/zstd/zstd_manual.html#Chapter4
  // When decompressing many times,
  // it is recommended to allocate a context only once,
  // and re-use it for each successive compression operation.
  // This will make workload friendlier for system's memory.
  // Use one context per thread for parallel execution.
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

  std::ifstream input(uri, std::ios::in | std::ios::binary);
  if (!input.is_open()) {
    std::stringstream errmsg;
    errmsg << "Failed to open file: \"" << uri <<
      "\" for binary reading! errno(" << errno << ")";

    throw std::runtime_error{errmsg.str()};
  }
  std::ofstream output(decompressed_uri, std::ios::out | std::ios::binary);
  if (!output.is_open()) {
    std::stringstream errmsg;
    errmsg << "Failed to open file: \"" << uri <<
      "\" for binary writing! errno(" << errno << ")";

    throw std::runtime_error{errmsg.str()};
  }
  // Base on the example from https://github.com/facebook/zstd/blob/dev/examples/streaming_decompression.c
  const size_t buff_in_size = ZSTD_DStreamInSize();
  const size_t buff_out_size = ZSTD_DStreamOutSize();
  size_t total_size = 0;
  std::vector<char> in_buffer(buff_in_size);
  std::vector<char> out_buffer(buff_out_size);
  size_t final_result = 0;
  do {
    input.read(in_buffer.data(), buff_in_size);
    const auto size = size_t(input.gcount());
    if (size > 0) {
      ZSTD_inBuffer z_in_buffer = {in_buffer.data(), static_cast<size_t>(size), 0};
      while (z_in_buffer.pos < z_in_buffer.size) {
        ZSTD_outBuffer z_out_buffer = {out_buffer.data(), out_buffer.size(), 0};
        const auto ret = ZSTD_decompressStream(zstd_context_, &z_out_buffer, &z_in_buffer);
        throw_on_zstd_error(ret);
        output.write(out_buffer.data(), z_out_buffer.pos);
        total_size += z_out_buffer.pos;
        final_result = ret;
      }
    }
  } while (!input.eof());
  output.flush();
  output.close();
  input.close();

  const auto end = std::chrono::high_resolution_clock::now();
  print_compression_statistics(start, end, final_result, total_size);

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
}  // namespace rosbag2_compression_zstd

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  rosbag2_compression_zstd::ZstdDecompressor,
  rosbag2_compression::BaseDecompressorInterface)
