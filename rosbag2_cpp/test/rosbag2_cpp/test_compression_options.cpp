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
  const auto compression_mode = rosbag2_cpp::compression_mode_from_string(compression_mode_string);
  EXPECT_EQ(compression_mode, rosbag2_cpp::CompressionMode::NONE);
}

TEST(CompressionOptionsFromStringTest, EmptyInputReturnsNoneMode)
{
  const std::string compression_mode_string;
  const auto compression_mode = rosbag2_cpp::compression_mode_from_string(compression_mode_string);
  EXPECT_EQ(compression_mode, rosbag2_cpp::CompressionMode::NONE);
}

TEST(CompressionOptionsFromStringTest, FileStringReturnsFileMode)
{
  const std::string compression_mode_string{"file"};
  const auto compression_mode = rosbag2_cpp::compression_mode_from_string(compression_mode_string);
  EXPECT_EQ(compression_mode, rosbag2_cpp::CompressionMode::FILE);
}

TEST(CompressionOptionsFromStringTest, MixedCaseMessageStringReturnsMessageMode)
{
  const std::string compression_mode_string{"MeSsAgE"};
  const auto compression_mode = rosbag2_cpp::compression_mode_from_string(compression_mode_string);
  EXPECT_EQ(compression_mode, rosbag2_cpp::CompressionMode::MESSAGE);
}

TEST(CompressionOptionsFromStringTest, MessageStringReturnsMessageMode)
{
  const std::string compression_mode_string{"MESSAGE"};
  const auto compression_mode = rosbag2_cpp::compression_mode_from_string(compression_mode_string);
  EXPECT_EQ(compression_mode, rosbag2_cpp::CompressionMode::MESSAGE);
}

TEST(CompressionOptionsToStringTest, BadModeReturnsNoneString)
{
  const auto compression_mode = static_cast<rosbag2_cpp::CompressionMode>(100);
  const auto compression_mode_string = rosbag2_cpp::compression_mode_to_string(compression_mode);
  EXPECT_EQ(compression_mode_string, "NONE");
}

TEST(CompressionOptionsToStringTest, MessageModeReturnsMessageString)
{
  const auto compression_mode = rosbag2_cpp::CompressionMode::MESSAGE;
  const auto compression_mode_string = rosbag2_cpp::compression_mode_to_string(compression_mode);
  EXPECT_EQ(compression_mode_string, "MESSAGE");
}

TEST(CompressionOptionsToStringTest, FileModeReturnsFileString)
{
  const auto compression_mode = rosbag2_cpp::CompressionMode::FILE;
  const auto compression_mode_string = rosbag2_cpp::compression_mode_to_string(compression_mode);
  EXPECT_EQ(compression_mode_string, "FILE");
}

TEST(CompressionOptionsToStringTest, NoneModeReturnsNoneString)
{
  const auto compression_mode = rosbag2_cpp::CompressionMode::NONE;
  const auto compression_mode_string = rosbag2_cpp::compression_mode_to_string(compression_mode);
  EXPECT_EQ(compression_mode_string, "NONE");
}
