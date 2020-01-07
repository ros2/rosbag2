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

#ifndef ROSBAG2_CPP__COMPRESSION_OPTIONS_HPP_
#define ROSBAG2_CPP__COMPRESSION_OPTIONS_HPP_

#include <string>

namespace rosbag2_cpp
{

enum class CompressionMode
{
  NONE,
  FILE,
  MESSAGE,
};

CompressionMode compression_mode_from_string(const std::string & compression_mode);

std::string compression_mode_to_string(CompressionMode compression_mode);

struct CompressionOptions
{
  std::string compression_format;
  CompressionMode compression_mode;
};

}  // namespace rosbag2_cpp
#endif  // ROSBAG2_CPP__COMPRESSION_OPTIONS_HPP_
