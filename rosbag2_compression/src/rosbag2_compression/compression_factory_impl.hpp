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

#include <memory>
#include <string>

#include "rosbag2_compression/compression_factory.hpp"

namespace rosbag2_compression
{

class CompressionFactoryImpl
{
public:
  CompressionFactoryImpl() = default;
  ~CompressionFactoryImpl() = default;

  std::unique_ptr<rosbag2_compression::BaseCompressorInterface>
  create_compressor(const std::string &)
  {
    return std::unique_ptr<rosbag2_compression::BaseCompressorInterface>();
  }

  std::unique_ptr<rosbag2_compression::BaseDecompressorInterface>
  create_decompressor(const std::string &)
  {
    return std::unique_ptr<rosbag2_compression::BaseDecompressorInterface>();
  }
};
}  // namespace rosbag2_compression

#endif  // ROSBAG2_COMPRESSION__COMPRESSION_FACTORY_IMPL_HPP_
