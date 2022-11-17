// Copyright 2022 Apex.AI, Inc.
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

#ifndef MSG_UTILS__MESSAGE_PRODUCER_FACTORY_HPP_
#define MSG_UTILS__MESSAGE_PRODUCER_FACTORY_HPP_

#include <memory>
#include <string>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/byte.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>
#include <std_msgs/msg/char.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/int8_multi_array.hpp>
#include <std_msgs/msg/multi_array_dimension.hpp>
#include <std_msgs/msg/multi_array_layout.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <std_msgs/msg/u_int64_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/range.hpp>

#include "message_producer.hpp"

#define ADD_MSG(space, type) \
  if (key == #space "::msg::"#type) { \
    using space::msg::type; \
    return std::make_shared<MessageProducer<type>>(args ...); \
  } else  // NOLINT  not closed `if` in macro is on purpose

#define ADD_STD_MSG(type) ADD_MSG(std_msgs, type)
#define ADD_SENSOR_MSG(type) ADD_MSG(sensor_msgs, type)

namespace msg_utils
{
template<typename ... Args>
std::shared_ptr<ProducerBase> create(std::string key, Args & ... args)
{
  ADD_STD_MSG(Bool)
  ADD_STD_MSG(Byte)
  ADD_STD_MSG(ByteMultiArray)
  ADD_STD_MSG(Char)
  ADD_STD_MSG(ColorRGBA)
  ADD_STD_MSG(Empty)
  ADD_STD_MSG(Float32)
  ADD_STD_MSG(Float32MultiArray)
  ADD_STD_MSG(Float64)
  ADD_STD_MSG(Float64MultiArray)
  ADD_STD_MSG(Header)
  ADD_STD_MSG(Int16)
  ADD_STD_MSG(Int16MultiArray)
  ADD_STD_MSG(Int32)
  ADD_STD_MSG(Int32MultiArray)
  ADD_STD_MSG(Int64)
  ADD_STD_MSG(Int64MultiArray)
  ADD_STD_MSG(Int8)
  ADD_STD_MSG(Int8MultiArray)
  ADD_STD_MSG(MultiArrayDimension)
  ADD_STD_MSG(MultiArrayLayout)
  ADD_STD_MSG(String)
  ADD_STD_MSG(UInt16)
  ADD_STD_MSG(UInt16MultiArray)
  ADD_STD_MSG(UInt32)
  ADD_STD_MSG(UInt32MultiArray)
  ADD_STD_MSG(UInt64)
  ADD_STD_MSG(UInt64MultiArray)
  ADD_STD_MSG(UInt8)
  ADD_STD_MSG(UInt8MultiArray)
  ADD_SENSOR_MSG(Image)
  ADD_SENSOR_MSG(PointCloud2)
  ADD_SENSOR_MSG(NavSatFix)
  ADD_SENSOR_MSG(Imu)
  ADD_SENSOR_MSG(Range)

  throw std::runtime_error("Unknown message type: " + key);
}

}  // namespace msg_utils

#endif  // MSG_UTILS__MESSAGE_PRODUCER_FACTORY_HPP_
