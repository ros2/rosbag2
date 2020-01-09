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

#ifndef ROSBAG2_CPP__COMPRESSION_OPTIONS_HPP_
#define ROSBAG2_CPP__COMPRESSION_OPTIONS_HPP_

#include <string>

#include "visibility_control.hpp"

namespace rosbag2_cpp
{

/**
 * Modes are used to specify whether to compress by individual serialized bag messages or by file.
 * rosbag2_cpp defaults to NONE.
 */
enum class ROSBAG2_CPP_PUBLIC CompressionMode : uint32_t
{
  NONE = 0,
  FILE,
  MESSAGE,
  LAST_MODE = MESSAGE
};

/**
 * Converts a string into a rosbag2_cpp::CompressionMode enum.
 *
 * \param compression_mode A case insensitive string that is either "FILE" or "MESSAGE".
 * \return CompressionMode NONE if compression_mode is invalid. FILE or MESSAGE otherwise.
 */
ROSBAG2_CPP_PUBLIC CompressionMode compression_mode_from_string(const std::string & compression_mode);

/**
 * Converts a rosbag2_cpp::CompressionMode enum into a string.
 *
 * \param compression_mode A CompressionMode enum.
 * \return The corresponding mode as a string.
 */
ROSBAG2_CPP_PUBLIC std::string compression_mode_to_string(CompressionMode compression_mode);

}  // namespace rosbag2_cpp
#endif  // ROSBAG2_CPP__COMPRESSION_OPTIONS_HPP_
