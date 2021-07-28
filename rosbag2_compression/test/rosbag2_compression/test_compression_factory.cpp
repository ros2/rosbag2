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

#include <gtest/gtest.h>

#include <string>
#include <algorithm>
#include <vector>

#include "rosbag2_compression/compression_factory.hpp"

class CompressionFactoryTest : public ::testing::Test
{
public:
  rosbag2_compression::CompressionFactory factory{};
};

TEST_F(CompressionFactoryTest, load_test_compressor) {
  const auto compression_format = "fake_comp";
  auto compressor = factory.create_compressor(compression_format);
  ASSERT_TRUE(compressor != nullptr);
  ASSERT_EQ(compression_format, compressor->get_compression_identifier());
}

TEST_F(CompressionFactoryTest, load_test_decompressor) {
  const auto compression_format = "fake_comp";
  auto decompressor = factory.create_decompressor(compression_format);
  ASSERT_TRUE(decompressor != nullptr);
  ASSERT_EQ(compression_format, decompressor->get_decompression_identifier());
}

TEST_F(CompressionFactoryTest, throws_on_bad_compressor_format) {
  const auto compression_format = "foo";
  ASSERT_EQ(factory.create_compressor(compression_format), nullptr);
}

TEST_F(CompressionFactoryTest, throws_on_bad_decompressor_format) {
  const auto compression_format = "bar";
  ASSERT_EQ(factory.create_decompressor(compression_format), nullptr);
}

TEST_F(CompressionFactoryTest, load_compression_plugins_test) {
  const auto compression_format = "fake_comp";
  auto compressor = factory.create_compressor(compression_format);
  ASSERT_TRUE(compressor != nullptr);
  std::vector<std::string> compressor_list = factory.get_declared_compressor_plugins();
  bool found_compressor = false;

  // Ensure the compressor plugin can be found
  for (auto it = compressor_list.begin(); it != compressor_list.end(); it++) {
    if (*it == compression_format) {
      found_compressor = true;
      break;
    }
  }
  ASSERT_TRUE(found_compressor);
}
