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

#ifndef ROSBAG2_COMPRESSION__MOCK_COMPRESSION_HPP_
#define ROSBAG2_COMPRESSION__MOCK_COMPRESSION_HPP_

#include <gmock/gmock.h>

#include <memory>
#include <string>

#include "rosbag2_compression/base_compressor_interface.hpp"
#include "rosbag2_compression/base_decompressor_interface.hpp"

class MockCompressor : public rosbag2_compression::BaseCompressorInterface
{
public:
  MOCK_METHOD1(compress_uri, std::string(const std::string & uri));
  MOCK_METHOD1(
    compress_serialized_bag_message,
    void(rosbag2_storage::SerializedBagMessage * bag_message));
  MOCK_CONST_METHOD0(get_compression_identifier, std::string());
};

class MockDecompressor : public rosbag2_compression::BaseDecompressorInterface
{
public:
  MOCK_METHOD1(decompress_uri, std::string(const std::string & uri));
  MOCK_METHOD1(
    decompress_serialized_bag_message,
    void(rosbag2_storage::SerializedBagMessage * bag_message));
  MOCK_CONST_METHOD0(get_decompression_identifier, std::string());
};

#endif  // ROSBAG2_COMPRESSION__MOCK_COMPRESSION_HPP_
