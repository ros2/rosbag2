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

#include <memory>
#include <string>

#include "rcpputils/filesystem_helper.hpp"

#include "gmock/gmock.h"

#include "fake_decompressor.hpp"

class FakeDecompressorTest : public ::testing::Test
{
public:
  FakeDecompressor decompressor;
};

TEST_F(FakeDecompressorTest, test_compress_file_method)
{
  rcpputils::fs::path test_path("/path/file.txt");
  auto compressed_uri = test_path.string() + "." + decompressor.get_decompression_identifier();
  auto expected_decompressed_uri = test_path.string();
  auto decompressed_uri = decompressor.decompress_uri(compressed_uri);
  EXPECT_EQ(decompressed_uri, expected_decompressed_uri);
}

TEST_F(FakeDecompressorTest, test_compress_bag_method)
{
  auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  decompressor.decompress_serialized_bag_message(bag_message.get());
  EXPECT_THAT(bag_message, ::testing::NotNull());
}
