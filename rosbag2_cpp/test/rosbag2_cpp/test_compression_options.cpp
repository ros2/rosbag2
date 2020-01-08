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

#include <gmock/gmock.h>

#include <string>

#include "rosbag2_cpp/compression_options.hpp"

TEST(CompressionOptionsFromStringTest, BadInputReturnsNoneMode)
{
  const std::string compression_mode_string{"bad_mode"};
  auto compression_mode = rosbag2_cpp::compression_mode_from_string(compression_mode_string);
  EXPECT_EQ(compression_mode, rosbag2_cpp::CompressionMode::NONE);
}

TEST(CompressionOptionsFromStringTest, EmptyInputReturnsNoneMode)
{
  const std::string compression_mode_string;
  auto compression_mode = rosbag2_cpp::compression_mode_from_string(compression_mode_string);
  EXPECT_EQ(compression_mode, rosbag2_cpp::CompressionMode::NONE);
}

TEST(CompressionOptionsFromStringTest, FileStringReturnsFileMode)
{
  std::string compression_mode_string{"file"};
  auto compression_mode = rosbag2_cpp::compression_mode_from_string(compression_mode_string);
  EXPECT_EQ(compression_mode, rosbag2_cpp::CompressionMode::FILE);
}

TEST(CompressionOptionsFromStringTest, MixedCaseMessageStringReturnsMessageMode)
{
  std::string compression_mode_string{"MeSsAgE"};
  auto compression_mode = rosbag2_cpp::compression_mode_from_string(compression_mode_string);
  EXPECT_EQ(compression_mode, rosbag2_cpp::CompressionMode::MESSAGE);
}

TEST(CompressionOptionsFromStringTest, MessageStringReturnsMessageMode)
{
  std::string compression_mode_string{"MESSAGE"};
  auto compression_mode = rosbag2_cpp::compression_mode_from_string(compression_mode_string);
  EXPECT_EQ(compression_mode, rosbag2_cpp::CompressionMode::MESSAGE);
}

TEST(CompressionOptionsToStringTest, BadModeReturnsNoneString)
{
  const auto compression_mode = static_cast<rosbag2_cpp::CompressionMode>(100);
  auto compression_mode_string = rosbag2_cpp::compression_mode_to_string(compression_mode);
  EXPECT_EQ(compression_mode_string, "NONE");
}

TEST(CompressionOptionsToStringTest, MessageModeReturnsMessageString)
{
  auto compression_mode = rosbag2_cpp::CompressionMode::MESSAGE;
  auto compression_mode_string = rosbag2_cpp::compression_mode_to_string(compression_mode);
  EXPECT_EQ(compression_mode_string, "MESSAGE");
}

TEST(CompressionOptionsToStringTest, FileModeReturnsFileString)
{
  auto compression_mode = rosbag2_cpp::CompressionMode::FILE;
  auto compression_mode_string = rosbag2_cpp::compression_mode_to_string(compression_mode);
  EXPECT_EQ(compression_mode_string, "FILE");
}

TEST(CompressionOptionsToStringTest, NoneModeReturnsNoneString)
{
  auto compression_mode = rosbag2_cpp::CompressionMode::NONE;
  auto compression_mode_string = rosbag2_cpp::compression_mode_to_string(compression_mode);
  EXPECT_EQ(compression_mode_string, "NONE");
}
