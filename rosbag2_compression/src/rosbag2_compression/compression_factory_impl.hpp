// Copyright 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#ifndef ROSBAG2_COMPRESSION__COMPRESSION_FACTORY_IMPL_HPP_
#define ROSBAG2_COMPRESSION__COMPRESSION_FACTORY_IMPL_HPP_

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>

#include "logging.hpp"
#include "rosbag2_compression/compression_factory.hpp"
#include "rosbag2_compression/zstd_compressor.hpp"
#include "rosbag2_compression/zstd_decompressor.hpp"

namespace
{

constexpr const char kCompressionFormatZstd[] = "zstd";

/// Convert a string to lowercase
std::string to_lower(const std::string & text)
{
  std::string lowercase_text = text;
  std::transform(lowercase_text.begin(), lowercase_text.end(), lowercase_text.begin(), ::tolower);
  return lowercase_text;
}
}  // namespace

namespace rosbag2_compression
{

/// Implementation of the CompressionFactory. See CompressionFactory for documentation.
class CompressionFactoryImpl
{
public:
  CompressionFactoryImpl() = default;
  ~CompressionFactoryImpl() = default;

  /// See CompressionFactory::create_compressor for documentation.
  std::unique_ptr<rosbag2_compression::BaseCompressorInterface>
  create_compressor(const std::string & compression_format)
  {
    const auto lowercase_format = to_lower(compression_format);
    if (lowercase_format == kCompressionFormatZstd) {
      return std::make_unique<rosbag2_compression::ZstdCompressor>();
    } else {
      std::stringstream errmsg;
      errmsg << "Compression format \"" << lowercase_format << "\" is not supported.";
      ROSBAG2_COMPRESSION_LOG_ERROR_STREAM(errmsg.str());
      throw std::invalid_argument(errmsg.str());
    }
  }

  /// See CompressionFactory::create_decompressor for documentation.
  std::unique_ptr<rosbag2_compression::BaseDecompressorInterface>
  create_decompressor(const std::string & compression_format)
  {
    const auto lowercase_format = to_lower(compression_format);
    if (lowercase_format == kCompressionFormatZstd) {
      return std::make_unique<rosbag2_compression::ZstdDecompressor>();
    } else {
      std::stringstream errmsg;
      errmsg << "Compression format \"" << lowercase_format << "\" is not supported.";
      ROSBAG2_COMPRESSION_LOG_ERROR_STREAM(errmsg.str());
      throw std::invalid_argument(errmsg.str());
    }
  }
};
}  // namespace rosbag2_compression

#endif  // ROSBAG2_COMPRESSION__COMPRESSION_FACTORY_IMPL_HPP_
