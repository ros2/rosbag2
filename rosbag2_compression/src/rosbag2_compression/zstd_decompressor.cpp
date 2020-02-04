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

#include "rcutils/filesystem.h"

#include "rosbag2_compression/zstd_decompressor.hpp"

#include "logging.hpp"

namespace
{

// String constant used to identify ZstdDecompressor.
constexpr const char kDecompressionIdentifier[] = "zstd";
constexpr const uint32_t kMagicNumberLittleEndian = 0xFD2FB528;
constexpr const uint32_t kMagicNumberBigEndian = 0x28B52FFD;

/**
 * Open a file using the C API.
 * This function calls OS-specific implementation of fopen.
 *
 * \param uri is the path to the file
 * \param read_mode is the read mode string accepted by fopen.
 * \return the FILE pointer or nullptr if the file was not opened.
 */
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

/**
 * Read a file from the supplied uri into a vector.
 *
 * \param uri is the path to the file.
 * \return the contents of the buffer as a vector.
 */
std::vector<uint8_t> get_input_buffer(const std::string & uri)
{
  // Read in buffer, handling accordingly
  const auto file_pointer = open_file(uri.c_str(), "rb");
  if (file_pointer == nullptr) {
    std::stringstream errmsg;
    errmsg << "Failed to open file: \"" << uri << "\" for binary reading!";

    throw std::runtime_error{errmsg.str()};
  }

  const auto compressed_buffer_length = rcutils_get_file_size(uri.c_str());
  if (compressed_buffer_length == 0) {
    fclose(file_pointer);

    std::stringstream errmsg;
    errmsg << "Unable to get size of file: " << uri;

    throw std::runtime_error{errmsg.str()};
  }

  // Initialize compress_buffer with size = compressed_buffer_length.
  // Uniform initialization cannot be used here since it will choose
  // the initializer list constructor instead.
  std::vector<uint8_t> compressed_buffer(compressed_buffer_length);

  const auto read_count = fread(
    compressed_buffer.data(), sizeof(decltype(compressed_buffer)::value_type),
    compressed_buffer_length, file_pointer);

  if (read_count != compressed_buffer_length) {
    ROSBAG2_COMPRESSION_LOG_ERROR_STREAM(
      "Bytes read !(" <<
        read_count << ") != compressed_buffer size (" << compressed_buffer.size() <<
        ")!");
    // An error indicator is set by fread, so the following check will throw.
  }

  if (ferror(file_pointer)) {
    fclose(file_pointer);

    std::stringstream errmsg;
    errmsg << "Unable to read binary data from file: \"" << uri << "\"!";

    throw std::runtime_error{errmsg.str()};
  }

  fclose(file_pointer);
  return compressed_buffer;
}

/**
 * Writes the output buffer to the specified file path.
 * \param output_buffer is the data to write.
 * \param uri is the relative file path to the output storage.
 */
void write_output_buffer(
  const std::vector<uint8_t> & output_buffer,
  const std::string & uri)
{
  if (output_buffer.empty()) {
    std::stringstream errmsg;
    errmsg << "Cannot write empty buffer to file: " << uri;

    throw std::runtime_error{errmsg.str()};
  }

  const auto file_pointer = open_file(uri.c_str(), "wb");
  if (file_pointer == nullptr) {
    std::stringstream errmsg;
    errmsg << "Failed to open file: \"" << uri << "\" for binary writing!";

    throw std::runtime_error{errmsg.str()};
  }

  const auto write_count = fwrite(
    output_buffer.data(), sizeof(uint8_t),
    output_buffer.size(), file_pointer);

  if (write_count != output_buffer.size()) {
    ROSBAG2_COMPRESSION_LOG_ERROR_STREAM(
      "Bytes written (" <<
        write_count << " != output_buffer size (" << output_buffer.size() <<
        ")!");
    // An error indicator is set by fwrite, so the following check will throw.
  }

  if (ferror(file_pointer)) {
    fclose(file_pointer);

    std::stringstream errmsg;
    errmsg << "Unable to write decompressed data to file: \"" << uri << "\"!";

    throw std::runtime_error{errmsg.str()};
  }

  fclose(file_pointer);
}

/**
 * Throws a runtime_error if data does not contain a ZSTD compressed frame.
 */
void throw_on_invalid_magic_number(const std::vector<uint8_t> & data)
{
  if (data.size() < 4) {
    throw std::runtime_error{"Data is too small to contain a ZSTD frame!"};
  }

  uint32_t magic_number = 0;
  auto ptr_magic_number = reinterpret_cast<uint8_t *>(&magic_number);
  ptr_magic_number[0] = data[0];
  ptr_magic_number[1] = data[1];
  ptr_magic_number[2] = data[2];
  ptr_magic_number[3] = data[3];

  bool is_little_endian = false;
  {
    uint16_t endian_check = 0x0001;
    auto ptr_endian_check = reinterpret_cast<uint8_t *>(&endian_check);

    // The set high byte would be 1 if system is big endian
    is_little_endian = (ptr_endian_check[0] == 0x01);
  }

  if (is_little_endian) {
    if (magic_number != kMagicNumberLittleEndian) {
      std::stringstream errmsg;
      errmsg << "Invalid magic number! Expected: 0x" <<
        std::hex << kMagicNumberLittleEndian <<
        " but got: 0x" << std::hex << magic_number;

      throw std::runtime_error{errmsg.str()};
    }
  } else {
    if (magic_number != kMagicNumberBigEndian) {
      std::stringstream errmsg;
      errmsg << "Invalid magic number! Expected: 0x" <<
        std::hex << kMagicNumberBigEndian <<
        " but got: 0x" << std::hex << magic_number;

      throw std::runtime_error{errmsg.str()};
    }
  }
}

/**
 * Checks compression_result and throws a runtime_error if there was a ZSTD error.
 */
void throw_on_zstd_error(const size_t compression_result)
{
  if (ZSTD_isError(compression_result)) {
    std::stringstream error;
    error << "ZSTD decompression error: " << ZSTD_getErrorName(compression_result);

    throw std::runtime_error{error.str()};
  }
}

/**
 * Checks frame_content and throws a runtime_error if there was a ZSTD error
 * or frame_content is invalid.
 */
void throw_on_invalid_frame_content(const size_t frame_content)
{
  if (frame_content == ZSTD_CONTENTSIZE_ERROR) {
    throw std::runtime_error{"Unable to determine file size due to error."};
  } else if (frame_content == ZSTD_CONTENTSIZE_UNKNOWN) {
    throw std::runtime_error{"Unable to determine file size."};
  }
}

/**
 * Prints decompression statistics to the debug log stream.
 * The log statement is formatted as JSON.
 * Time is formatted as a decimal of seconds.
 *
 * Example:
 *  "Decompression statistics" : {"Time" : 1.2, "Compression Ratio" : 0.5}
 *
 * \param start is the time_point when compression started.
 * \param end is the time_point when compression ended.
 * \param decompressed_size is the file size after decompression
 * \param compressed_size is the compressed file size
 */
void print_decompression_statistics(
  const std::chrono::high_resolution_clock::time_point start,
  const std::chrono::high_resolution_clock::time_point end,
  const size_t decompressed_size,
  const size_t compressed_size)
{
  const auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  const auto decompression_ratio =
    static_cast<double>(decompressed_size) / static_cast<double>(compressed_size);

  ROSBAG2_COMPRESSION_LOG_DEBUG_STREAM(
    "\"Decompression statistics\" : {" <<
      "\"Time\" : " << (duration.count() / 1000.0) <<
      ", \"Decompression Ratio\" : " << decompression_ratio <<
      "}");
}
}  // namespace

namespace rosbag2_compression
{

std::string ZstdDecompressor::decompress_uri(const std::string & uri)
{
  const auto start = std::chrono::high_resolution_clock::now();
  const auto uri_path = rcpputils::fs::path{uri};
  const auto decompressed_uri = rcpputils::fs::remove_extension(uri_path).string();
  const auto compressed_buffer = get_input_buffer(uri);
  throw_on_invalid_magic_number(compressed_buffer);
  const auto compressed_buffer_length = compressed_buffer.size();

  const auto decompressed_buffer_length =
    ZSTD_getFrameContentSize(compressed_buffer.data(), compressed_buffer_length);

  throw_on_invalid_frame_content(decompressed_buffer_length);

  // Initializes decompressed_buffer with size = decompressed_buffer_length.
  // Uniform initialization cannot be used here since it will choose
  // the initializer list constructor instead.
  std::vector<uint8_t> decompressed_buffer(decompressed_buffer_length);

  const auto decompression_result = ZSTD_decompress(
    decompressed_buffer.data(), decompressed_buffer_length,
    compressed_buffer.data(), compressed_buffer_length);

  throw_on_zstd_error(decompression_result);
  decompressed_buffer.resize(decompression_result);

  write_output_buffer(decompressed_buffer, decompressed_uri);

  const auto end = std::chrono::high_resolution_clock::now();
  print_decompression_statistics(start, end, decompression_result, compressed_buffer_length);

  return decompressed_uri;
}

void ZstdDecompressor::decompress_serialized_bag_message(
  rosbag2_storage::SerializedBagMessage *)
{
  throw std::logic_error{"Not implemented"};
}

std::string ZstdDecompressor::get_decompression_identifier() const
{
  return kDecompressionIdentifier;
}
}  // namespace rosbag2_compression
