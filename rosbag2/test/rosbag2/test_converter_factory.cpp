// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#include <memory>
#include <string>

#include "rosbag2/serialization_format_converter_factory.hpp"

using rosbag2::SerializationFormatConverterInterface;

class ConverterFactoryTest : public ::testing::Test
{
public:
  rosbag2::SerializationFormatConverterFactory factory;
};

TEST_F(ConverterFactoryTest, load_test_plugin) {
  auto converter = factory.load_deserializer("a");
  ASSERT_NE(nullptr, converter);
}

TEST_F(ConverterFactoryTest, load_converrters_returns_nullptr_if_plugin_does_not_exist) {
  auto converter = factory.load_deserializer("wrong_format");
  ASSERT_EQ(nullptr, converter);
}
