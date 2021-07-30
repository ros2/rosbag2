// Copyright 2021 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#ifndef ROSBAG2_COMPRESSION__COMPRESSION_TRAITS_HPP_
#define ROSBAG2_COMPRESSION__COMPRESSION_TRAITS_HPP_

#include <string>

namespace rosbag2_compression
{
  template<typename T>
  struct CompressionTraits
  {};

  template<>
  struct CompressionTraits<BaseCompressorInterface>
  {
  static constexpr const char * name = "rosbag2_compression::BaseCompressorInterface";
  };

  template<>
  struct CompressionTraits<BaseDecompressorInterface>
  {
  static constexpr const char * name = "rosbag2_compression::BaseDecompressorInterface";
  };
}  // namespace rosbag2_compression
#endif  // ROSBAG2_COMPRESSION__COMPRESSION_TRAITS_HPP_
