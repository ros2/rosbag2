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

#include <string>

#include "rosbag2_cpp/compression_options.hpp"
#include "rosbag2_cpp/logging.hpp"

namespace rosbag2_cpp
{

namespace
{

  constexpr const char kCompressionModeNoneStr[] = "NONE";
  constexpr const char kCompressionModeFileStr[] = "FILE";
  constexpr const char kCompressionModeMessageStr[] = "MESSAGE";

}  // namespace

CompressionMode compression_mode_from_string(const std::string & compression_mode)
{
  if (compression_mode.empty() || compression_mode == kCompressionModeNoneStr) {
    return CompressionMode::NONE;
  } else if (compression_mode == kCompressionModeFileStr) {
    return CompressionMode::FILE;
  } else if (compression_mode == kCompressionModeMessageStr) {
    return CompressionMode::MESSAGE;
  } else {
    ROSBAG2_CPP_LOG_ERROR_STREAM(
      "CompressionMode: \"" << compression_mode << "\" is not supported!");
    return CompressionMode::NONE;
  }
}

std::string compression_mode_to_string(const CompressionMode compression_mode)
{
  switch (compression_mode) {
    case CompressionMode::NONE:
      return kCompressionModeNoneStr;
    case CompressionMode::FILE:
      return kCompressionModeFileStr;
    case CompressionMode::MESSAGE:
      return kCompressionModeMessageStr;
    default:
      ROSBAG2_CPP_LOG_ERROR_STREAM("CompressionMode not supported!");
      return kCompressionModeNoneStr;
  }
}
}  // namespace rosbag2_cpp
