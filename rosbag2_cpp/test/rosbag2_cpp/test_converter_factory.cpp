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

#include <gmock/gmock.h>

#include <memory>
#include <string>

#include "rosbag2_cpp/serialization_format_converter_factory.hpp"

using namespace ::testing;  // NOLINT

class ConverterFactoryTest : public Test
{
public:
  rosbag2_cpp::SerializationFormatConverterFactory factory;
};

TEST_F(ConverterFactoryTest,
  load_test_plugin_can_load_a_converter_plugin_as_both_deserializer_and_serializer)
{
  auto deserializer = factory.load_deserializer("a");
  EXPECT_THAT(deserializer, NotNull());
  auto serializer = factory.load_serializer("a");
  EXPECT_THAT(serializer, NotNull());
}

TEST_F(ConverterFactoryTest, load_converters_returns_nullptr_if_plugin_does_not_exist) {
  auto converter = factory.load_deserializer("wrong_format");
  EXPECT_THAT(converter, IsNull());
}

TEST_F(ConverterFactoryTest, load_test_serializer_plugin) {
  auto converter = factory.load_serializer("s");
  EXPECT_THAT(converter, NotNull());
}

TEST_F(ConverterFactoryTest, load_deserializer_cannot_load_serializer_plugin) {
  auto converter = factory.load_deserializer("s");
  EXPECT_THAT(converter, IsNull());
}
