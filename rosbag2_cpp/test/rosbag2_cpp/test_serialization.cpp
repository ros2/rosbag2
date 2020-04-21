// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/byte.hpp"
#include "std_msgs/msg/char.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/u_int64.hpp"

#include "rosbag2_cpp/deserializer.hpp"
#include "rosbag2_cpp/serializer.hpp"

#include "test_msgs/message_fixtures.hpp"


using namespace ::testing;  // NOLINT
using namespace rosbag2_cpp;  // NOLINT

const char * const kGenericTopic = "generic";

TEST(SerializeDeserialize, basic_types) {
  Deserializer deserializer;
  Serializer serializer;

  for (test_msgs::msg::BasicTypes::SharedPtr msg : get_messages_basic_types()) {
    // Check all the message components
    std_msgs::msg::builder::Init_Bool_data boolBuilder;
    std_msgs::msg::Bool boolValue = boolBuilder.data(msg->bool_value);
    EXPECT_EQ(
      deserializer.deserialize<std_msgs::msg::Bool>(
        serializer.serialize(boolValue, kGenericTopic)),
      boolValue);
    std_msgs::msg::builder::Init_Byte_data byteBuilder;
    std_msgs::msg::Byte byteValue = byteBuilder.data(msg->byte_value);
    EXPECT_EQ(
      deserializer.deserialize<std_msgs::msg::Byte>(
        serializer.serialize(byteValue, kGenericTopic)),
      byteValue);
    std_msgs::msg::builder::Init_Char_data charBuilder;
    std_msgs::msg::Char charValue = charBuilder.data(msg->char_value);
    EXPECT_EQ(
      deserializer.deserialize<std_msgs::msg::Char>(
        serializer.serialize(charValue, kGenericTopic)),
      charValue);
    std_msgs::msg::builder::Init_Float32_data float32Builder;
    std_msgs::msg::Float32 float32Value = float32Builder.data(msg->float32_value);
    EXPECT_EQ(
      deserializer.deserialize<std_msgs::msg::Float32>(
        serializer.serialize(float32Value, kGenericTopic)),
      float32Value);
    std_msgs::msg::builder::Init_Float64_data float64Builder;
    std_msgs::msg::Float64 float64Value = float64Builder.data(msg->float64_value);
    EXPECT_EQ(
      deserializer.deserialize<std_msgs::msg::Float64>(
        serializer.serialize(float64Value, kGenericTopic)),
      float64Value);
    std_msgs::msg::builder::Init_Int8_data int8Builder;
    std_msgs::msg::Int8 int8Value = int8Builder.data(msg->int8_value);
    EXPECT_EQ(
      deserializer.deserialize<std_msgs::msg::Int8>(
        serializer.serialize(int8Value, kGenericTopic)),
      int8Value);
    std_msgs::msg::builder::Init_UInt8_data uint8Builder;
    std_msgs::msg::UInt8 uint8Value = uint8Builder.data(msg->uint8_value);
    EXPECT_EQ(
      deserializer.deserialize<std_msgs::msg::UInt8>(
        serializer.serialize(uint8Value, kGenericTopic)),
      uint8Value);
    std_msgs::msg::builder::Init_Int16_data int16Builder;
    std_msgs::msg::Int16 int16Value = int16Builder.data(msg->int16_value);
    EXPECT_EQ(
      deserializer.deserialize<std_msgs::msg::Int16>(
        serializer.serialize(int16Value, kGenericTopic)),
      int16Value);
    std_msgs::msg::builder::Init_UInt16_data uint16Builder;
    std_msgs::msg::UInt16 uint16Value = uint16Builder.data(msg->uint16_value);
    EXPECT_EQ(
      deserializer.deserialize<std_msgs::msg::UInt16>(
        serializer.serialize(uint16Value, kGenericTopic)),
      uint16Value);
    std_msgs::msg::builder::Init_Int32_data int32Builder;
    std_msgs::msg::Int32 int32Value = int32Builder.data(msg->int32_value);
    EXPECT_EQ(
      deserializer.deserialize<std_msgs::msg::Int32>(
        serializer.serialize(int32Value, kGenericTopic)),
      int32Value);
    std_msgs::msg::builder::Init_UInt32_data uint32Builder;
    std_msgs::msg::UInt32 uint32Value = uint32Builder.data(msg->uint32_value);
    EXPECT_EQ(
      deserializer.deserialize<std_msgs::msg::UInt32>(
        serializer.serialize(uint32Value, kGenericTopic)),
      uint32Value);
    std_msgs::msg::builder::Init_Int64_data int64Builder;
    std_msgs::msg::Int64 int64Value = int64Builder.data(msg->int64_value);
    EXPECT_EQ(
      deserializer.deserialize<std_msgs::msg::Int64>(
        serializer.serialize(int64Value, kGenericTopic)),
      int64Value);
    std_msgs::msg::builder::Init_UInt64_data uint64Builder;
    std_msgs::msg::UInt64 uint64Value = uint64Builder.data(msg->uint64_value);
    EXPECT_EQ(
      deserializer.deserialize<std_msgs::msg::UInt64>(
        serializer.serialize(uint64Value, kGenericTopic)),
      uint64Value);

    // Check the full message end to end
    EXPECT_EQ(
      deserializer.deserialize<test_msgs::msg::BasicTypes>(
        serializer.serialize<test_msgs::msg::BasicTypes>(*msg, kGenericTopic)),
      *msg);
  }
}
