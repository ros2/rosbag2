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

#include "rosbag2_compression/compression_factory.hpp"

namespace
{

std::string to_lower(const std::string & text)
{
  std::string lowercase_text = text;
  std::transform(lowercase_text.begin(), lowercase_text.end(), lowercase_text.begin(), ::tolower);
  return lowercase_text;
}
}  // namespace

class CompressionFactoryTest : public ::testing::Test
{
public:
  rosbag2_compression::CompressionFactory factory{};
};

TEST_F(CompressionFactoryTest, creates_zstd_compressor) {
  const auto compression_format = "zstd";
  auto zstd_compressor = factory.create_compressor(compression_format);
  ASSERT_EQ(compression_format, zstd_compressor->get_compression_identifier());
}

TEST_F(CompressionFactoryTest, creates_zstd_compressor_caseinsensitive)
{
  const auto compression_format = "ZsTd";
  const auto lowercase_compression_format = to_lower(compression_format);
  auto zstd_compressor = factory.create_compressor(compression_format);
  ASSERT_EQ(lowercase_compression_format, zstd_compressor->get_compression_identifier());
}

TEST_F(CompressionFactoryTest, creates_zstd_compressor_uppercase)
{
  const auto compression_format = "ZSTD";
  const auto lowercase_compression_format = to_lower(compression_format);
  auto zstd_compressor = factory.create_compressor(compression_format);
  ASSERT_EQ(lowercase_compression_format, zstd_compressor->get_compression_identifier());
}

TEST_F(CompressionFactoryTest, creates_zstd_decompressor) {
  const auto compression_format = "zstd";
  auto zstd_decompressor = factory.create_decompressor(compression_format);
  ASSERT_EQ(compression_format, zstd_decompressor->get_decompression_identifier());
}

TEST_F(CompressionFactoryTest, creates_zstd_decompressor_caseinsensitive)
{
  const auto compression_format = "ZsTd";
  const auto lowercase_compression_format = to_lower(compression_format);
  auto zstd_compressor = factory.create_decompressor(compression_format);
  ASSERT_EQ(lowercase_compression_format, zstd_compressor->get_decompression_identifier());
}

TEST_F(CompressionFactoryTest, creates_zstd_decompressor_uppercase)
{
  const auto compression_format = "ZSTD";
  const auto lowercase_compression_format = to_lower(compression_format);
  auto zstd_compressor = factory.create_decompressor(compression_format);
  ASSERT_EQ(lowercase_compression_format, zstd_compressor->get_decompression_identifier());
}

TEST_F(CompressionFactoryTest, throws_on_bad_compressor_format) {
  const auto compression_format = "foo";
  ASSERT_THROW(factory.create_compressor(compression_format), std::invalid_argument);
}

TEST_F(CompressionFactoryTest, throws_on_bad_decompressor_format) {
  const auto compression_format = "bar";
  ASSERT_THROW(factory.create_compressor(compression_format), std::invalid_argument);
}
