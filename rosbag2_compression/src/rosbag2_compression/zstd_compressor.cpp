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
  // From the zstd manual: https://facebook.github.io/zstd/zstd_manual.html#Chapter4
  // When compressing many times,
  // it is recommended to allocate a context just once,
  // and re-use it for each successive compression operation.
  // This will make workload friendlier for system's memory.
  // Note : re-using context is just a speed / resource optimization.
  //        It doesn't change the compression ratio, which remains identical.
  // Note 2 : In multi-threaded environments,
  //        use one different context per thread for parallel execution.
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

  std::ifstream input(uri, std::ios::in | std::ios::binary);
  if (!input.is_open()) {
    std::stringstream errmsg;
    errmsg << "Failed to open file: \"" << uri <<
      "\" for binary reading! errno(" << errno << ")";

    throw std::runtime_error{errmsg.str()};
  }
  std::ofstream output(compressed_uri, std::ios::out | std::ios::binary);
  if (!output.is_open()) {
    std::stringstream errmsg;
    errmsg << "Failed to open file: \"" << uri <<
      "\" for binary writing! errno(" << errno << ")";

    throw std::runtime_error{errmsg.str()};
  }
  // Based on the example from https://github.com/facebook/zstd/blob/dev/examples/streaming_compression.c
  const size_t buff_in_size = ZSTD_CStreamInSize();
  const size_t buff_out_size = ZSTD_CStreamOutSize();
  std::vector<char> in_buffer(buff_in_size);
  std::vector<char> out_buffer(buff_out_size);
  size_t total_size = 0;
  size_t final_result = 0;
  do {
    input.read(in_buffer.data(), buff_in_size);
    const auto size = size_t(input.gcount());
    if (size > 0) {
      const ZSTD_EndDirective mode = input.eof() ? ZSTD_e_end : ZSTD_e_continue;
      ZSTD_inBuffer z_in_buffer = {in_buffer.data(), static_cast<size_t>(size), 0};
      int finished;
      do {
        ZSTD_outBuffer z_out_buffer = {out_buffer.data(), out_buffer.size(), 0};
        const auto remaining =
          ZSTD_compressStream2(zstd_context_, &z_out_buffer, &z_in_buffer, mode);
        throw_on_zstd_error(remaining);
        output.write(out_buffer.data(), z_out_buffer.pos);
        total_size += z_out_buffer.pos;
        finished = input.eof() ? (remaining == 0) : (z_in_buffer.pos == z_in_buffer.size);
        final_result = remaining;
      } while (!finished);
    }
  } while (!input.eof());
  output.flush();
  output.close();
  input.close();

  const auto end = std::chrono::high_resolution_clock::now();
  print_compression_statistics(start, end, total_size, final_result);
  return compressed_uri;
}

void ZstdCompressor::compress_serialized_bag_message(
  rosbag2_storage::SerializedBagMessage * message)
{
  const auto start = std::chrono::high_resolution_clock::now();
  // Allocate based on compression bound and compress
  const auto maximum_compressed_length =
    ZSTD_compressBound(message->serialized_data->buffer_length);
  std::vector<uint8_t> compressed_buffer(maximum_compressed_length);

  // Perform compression and check.
  // compression_result is either the actual compressed size or an error code.
  const auto compression_result = ZSTD_compressCCtx(
    zstd_context_,
    compressed_buffer.data(), maximum_compressed_length,
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
  print_compression_statistics(start, end, maximum_compressed_length, compression_result);
}

std::string ZstdCompressor::get_compression_identifier() const
{
  return kCompressionIdentifier;
}

}  // namespace rosbag2_compression

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  rosbag2_compression::ZstdCompressor,
  rosbag2_compression::BaseCompressorInterface)
