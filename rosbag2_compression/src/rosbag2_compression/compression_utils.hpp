// Copyright 2020 Southwest Research Institute. All Rights Reserved.
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

#ifndef ROSBAG2_COMPRESSION__COMPRESSION_UTILS_HPP_
#define ROSBAG2_COMPRESSION__COMPRESSION_UTILS_HPP_

#include <zstd.h>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "logging.hpp"

#include "rcpputils/filesystem_helper.hpp"

namespace rosbag2_compression
{
// Increasing the compression level will:
//   - Increase the time taken to compress
//   - Decrease the size of the compressed data
// Setting to zero uses Zstd's default value of 3.
constexpr const int kDefaultZstdCompressionLevel = 1;
// String constant used to identify ZstdCompressor.
constexpr const char kCompressionIdentifier[] = "zstd";
// String constant used to identify ZstdDecompressor.
constexpr const char kDecompressionIdentifier[] = "zstd";
// Used as a parameter type in a function that accepts the output of ZSTD_compress.
using ZstdCompressReturnType = decltype(ZSTD_compress(
    nullptr, 0,
    nullptr, 0, 0));
// Used as a parameter type in a function that accepts the output of ZSTD_decompress.
using ZstdDecompressReturnType = decltype(ZSTD_decompress(
    nullptr, 0,
    nullptr, 0));
// Used as a parameter type in a function that accepts the output of ZSTD_getFrameContentSize.
using ZstdGetFrameContentSizeReturnType = decltype(ZSTD_getFrameContentSize(nullptr, 0));

/**
 * Read a file from the supplied uri into a vector.
 *
 * \param uri is the path to the file.
 * \return the contents of the buffer as a vector.
 */
std::vector<uint8_t> get_input_buffer(const std::string & uri);

/**
 * Writes the output buffer to the specified file path.
 * \param output_buffer is the data to write.
 * \param uri is the relative file path to the output storage.
 */
void write_output_buffer(
  const std::vector<uint8_t> & output_buffer,
  const std::string & uri);

/**
 * Checks compression_result and throws a runtime_error if there was a ZSTD error.
 * \param compression_result is the return value of ZSTD_compress or ZSTD_decompress.
 */
void throw_on_zstd_error(const ZstdDecompressReturnType compression_result);

/**
 * Checks rcutils array resizing and throws a runtime_error if there was an error resizing.
 * \param rcutils_ret_t Result of calling rcutils
 */
void throw_on_rcutils_resize_error(const rcutils_ret_t resize_result);

/**
 * Checks frame_content and throws a runtime_error if there was a ZSTD error
 * or frame_content is invalid.
 * \param frame_content is the return value of ZSTD_getFrameContentSize.
 */
void throw_on_invalid_frame_content(const ZstdGetFrameContentSizeReturnType frame_content);

/**
 * Prints compression statistics to the debug log stream.
 * The log statement is formatted as JSON.
 * Time is formatted as a decimal of seconds.
 *
 * Example:
 *  "Compression statistics" : {"Time" : 1.2, "Compression Ratio" : 0.5}
 *
 * \param start is the time_point when compression or decompression started.
 * \param end is the time_point when compression or decompression ended.
 * \param decompressed_size is the decompressed data size
 * \param compressed_size is the compressed data size
 */
void print_compression_statistics(
  const std::chrono::high_resolution_clock::time_point start,
  const std::chrono::high_resolution_clock::time_point end,
  const size_t decompressed_size,
  const size_t compressed_size);
}  // namespace rosbag2_compression

#endif  // ROSBAG2_COMPRESSION__COMPRESSION_UTILS_HPP_
