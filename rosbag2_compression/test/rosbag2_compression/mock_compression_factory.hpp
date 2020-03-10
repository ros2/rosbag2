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

#ifndef ROSBAG2_COMPRESSION__MOCK_COMPRESSION_FACTORY_HPP_
#define ROSBAG2_COMPRESSION__MOCK_COMPRESSION_FACTORY_HPP_

#include <gmock/gmock.h>

#include <memory>
#include <string>

#include "rosbag2_compression/compression_factory.hpp"

class MockCompressionFactory : public rosbag2_compression::CompressionFactory
{
public:
  MOCK_METHOD1(
    create_compressor,
    std::unique_ptr<rosbag2_compression::BaseCompressorInterface>(
      const std::string &));

  MOCK_METHOD1(
    create_decompressor,
    std::unique_ptr<rosbag2_compression::BaseDecompressorInterface>(
      const std::string & compression_format));
};

#endif  // ROSBAG2_COMPRESSION__MOCK_COMPRESSION_FACTORY_HPP_
