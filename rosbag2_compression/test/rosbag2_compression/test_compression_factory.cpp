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

#include "rosbag2_compression/compression_factory.hpp"


TEST(CompressionFactoryTest, creates_zstd_compressor) {
  rosbag2_compression::CompressionFactory factory{};
  const auto compression_format = "zstd";
  auto zstd_compressor = factory.create_compressor(compression_format);
  ASSERT_EQ(compression_format, zstd_compressor->get_compression_identifier());
}

TEST(CompressionFactoryTest, creates_zstd_decompressor) {
  rosbag2_compression::CompressionFactory factory{};
  const auto compression_format = "zstd";
  auto zstd_decompressor = factory.create_compressor(compression_format);
  ASSERT_EQ(compression_format, zstd_decompressor->get_compression_identifier());
}

TEST(CompressionFactoryTest, throws_on_bad_compressor_format) {
  rosbag2_compression::CompressionFactory factory{};
  const auto compression_format = "foo";
  ASSERT_THROW(factory.create_compressor(compression_format), std::invalid_argument);
}

TEST(CompressionFactoryTest, throws_on_bad_decompressor_format) {
  rosbag2_compression::CompressionFactory factory{};
  const auto compression_format = "bar";
  ASSERT_THROW(factory.create_compressor(compression_format), std::invalid_argument);
}
