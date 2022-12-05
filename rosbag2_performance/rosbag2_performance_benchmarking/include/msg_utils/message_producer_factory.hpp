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

#include <rosbag2_performance_benchmarking_msgs/msg/byte_array.hpp>

#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/channel_float32.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/illuminance.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joy_feedback.hpp>
#include <sensor_msgs/msg/joy_feedback_array.hpp>
#include <sensor_msgs/msg/laser_echo.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/multi_dof_joint_state.hpp>
#include <sensor_msgs/msg/multi_echo_laser_scan.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>
#include <sensor_msgs/msg/relative_humidity.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/time_reference.hpp>

#include "message_producer.hpp"

#define IF_KEY_MATCH_TYPE_CREATE_MSG_PRODUCER_ELSE(space, type) \
  if (key == #space "::msg::"#type) { \
    using space::msg::type; \
    return std::make_shared<MessageProducer<type>>(args ...); \
  } else  // NOLINT  not closed `if` in macro is on purpose

#define IF_KEY_MATCH_TYPE_CREATE_BENCHMARKING_MSG_PRODUCER_ELSE(type) \
  IF_KEY_MATCH_TYPE_CREATE_MSG_PRODUCER_ELSE(rosbag2_performance_benchmarking_msgs, type)
#define IF_KEY_MATCH_TYPE_CREATE_SENSOR_MSG_PRODUCER_ELSE(type) \
  IF_KEY_MATCH_TYPE_CREATE_MSG_PRODUCER_ELSE(sensor_msgs, type)

namespace msg_utils
{
template<typename ... Args>
std::shared_ptr<ProducerBase> create(std::string key, Args & ... args)
{
  IF_KEY_MATCH_TYPE_CREATE_BENCHMARKING_MSG_PRODUCER_ELSE(ByteArray)

  IF_KEY_MATCH_TYPE_CREATE_SENSOR_MSG_PRODUCER_ELSE(BatteryState)
  IF_KEY_MATCH_TYPE_CREATE_SENSOR_MSG_PRODUCER_ELSE(CameraInfo)
  IF_KEY_MATCH_TYPE_CREATE_SENSOR_MSG_PRODUCER_ELSE(ChannelFloat32)
  IF_KEY_MATCH_TYPE_CREATE_SENSOR_MSG_PRODUCER_ELSE(CompressedImage)
  IF_KEY_MATCH_TYPE_CREATE_SENSOR_MSG_PRODUCER_ELSE(FluidPressure)
  IF_KEY_MATCH_TYPE_CREATE_SENSOR_MSG_PRODUCER_ELSE(Illuminance)
  IF_KEY_MATCH_TYPE_CREATE_SENSOR_MSG_PRODUCER_ELSE(Image)
  IF_KEY_MATCH_TYPE_CREATE_SENSOR_MSG_PRODUCER_ELSE(Imu)
  IF_KEY_MATCH_TYPE_CREATE_SENSOR_MSG_PRODUCER_ELSE(JointState)
  IF_KEY_MATCH_TYPE_CREATE_SENSOR_MSG_PRODUCER_ELSE(Joy)
  IF_KEY_MATCH_TYPE_CREATE_SENSOR_MSG_PRODUCER_ELSE(JoyFeedbackArray)
  IF_KEY_MATCH_TYPE_CREATE_SENSOR_MSG_PRODUCER_ELSE(LaserEcho)
  IF_KEY_MATCH_TYPE_CREATE_SENSOR_MSG_PRODUCER_ELSE(LaserScan)
  IF_KEY_MATCH_TYPE_CREATE_SENSOR_MSG_PRODUCER_ELSE(MagneticField)
  IF_KEY_MATCH_TYPE_CREATE_SENSOR_MSG_PRODUCER_ELSE(MultiDOFJointState)
  IF_KEY_MATCH_TYPE_CREATE_SENSOR_MSG_PRODUCER_ELSE(MultiEchoLaserScan)
  IF_KEY_MATCH_TYPE_CREATE_SENSOR_MSG_PRODUCER_ELSE(NavSatFix)
  IF_KEY_MATCH_TYPE_CREATE_SENSOR_MSG_PRODUCER_ELSE(PointCloud)
  IF_KEY_MATCH_TYPE_CREATE_SENSOR_MSG_PRODUCER_ELSE(PointCloud2)
  IF_KEY_MATCH_TYPE_CREATE_SENSOR_MSG_PRODUCER_ELSE(PointField)
  IF_KEY_MATCH_TYPE_CREATE_SENSOR_MSG_PRODUCER_ELSE(Range)
  IF_KEY_MATCH_TYPE_CREATE_SENSOR_MSG_PRODUCER_ELSE(RelativeHumidity)
  IF_KEY_MATCH_TYPE_CREATE_SENSOR_MSG_PRODUCER_ELSE(Temperature)
  IF_KEY_MATCH_TYPE_CREATE_SENSOR_MSG_PRODUCER_ELSE(TimeReference)

  throw std::runtime_error("Unknown message type: " + key);
}

}  // namespace msg_utils

#endif  // MSG_UTILS__MESSAGE_PRODUCER_FACTORY_HPP_
