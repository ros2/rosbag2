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

#include "compression_utils.hpp"

#include <string>
#include <vector>

namespace
{
/**
 * Open a file using the OS-specific C API.
 * \param uri is the path to the file.
 * \param read_mode is the read mode accepted by OS-specific fopen.
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
}  // namespace

namespace rosbag2_compression
{

void write_output_buffer(
  const std::vector<uint8_t> & output_buffer,
  const std::string & uri)
{
  if (output_buffer.empty()) {
    std::stringstream errmsg;
    errmsg << "Cannot write empty buffer to file: \"" << uri << "\"";

    throw std::runtime_error{errmsg.str()};
  }

  const auto file_pointer = open_file(uri, "wb");
  if (file_pointer == nullptr) {
    std::stringstream errmsg;
    errmsg << "Failed to open file: \"" << uri <<
      "\" for binary writing! errno(" << errno << ")";

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
    errmsg << "Unable to write data to file: \"" << uri << "\"!";

    throw std::runtime_error{errmsg.str()};
  }

  fclose(file_pointer);
}


void throw_on_zstd_error(const ZstdDecompressReturnType compression_result)
{
  if (ZSTD_isError(compression_result)) {
    std::stringstream error;
    error << "ZSTD decompression error: " << ZSTD_getErrorName(compression_result);

    throw std::runtime_error{error.str()};
  }
}

void throw_on_rcutils_resize_error(const rcutils_ret_t resize_result)
{
  if (resize_result == RCUTILS_RET_OK) {
    return;
  }

  std::stringstream error;
  error << "rcutils_uint8_array_resize error: ";
  switch (resize_result) {
    case RCUTILS_RET_INVALID_ARGUMENT:
      error << "Invalid Argument";
      break;
    case RCUTILS_RET_BAD_ALLOC:
      error << "Bad Alloc";
      break;
    case RCUTILS_RET_ERROR:
      error << "Ret Error";
      break;
    default:
      error << "Unexpected Result";
      break;
  }
  throw std::runtime_error(error.str());
}

void throw_on_invalid_frame_content(const ZstdGetFrameContentSizeReturnType frame_content)
{
  if (frame_content == ZSTD_CONTENTSIZE_ERROR) {
    throw std::runtime_error{"Unable to determine file size due to error."};
  } else if (frame_content == ZSTD_CONTENTSIZE_UNKNOWN) {
    throw std::runtime_error{"Unable to determine file size."};
  }
}

void print_compression_statistics(
  const std::chrono::high_resolution_clock::time_point start,
  const std::chrono::high_resolution_clock::time_point end,
  const size_t decompressed_size,
  const size_t compressed_size)
{
  const auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  const auto decompression_ratio =
    static_cast<double>(decompressed_size) / static_cast<double>(compressed_size);

  ROSBAG2_COMPRESSION_LOG_DEBUG_STREAM(
    "\"Compression statistics\" : {" <<
      "\"Time\" : " << (duration.count() / 1000.0) <<
      ", \"Compression Ratio\" : " << decompression_ratio <<
      "}");
}
}  // namespace rosbag2_compression
