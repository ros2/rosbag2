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

#define ADD_MSG(space, type) \
  if (key == #space "::msg::"#type) { \
    using space::msg::type; \
    return std::make_shared<MessageProducer<type>>(args ...); \
  } else  // NOLINT  not closed `if` in macro is on purpose

#define ADD_BENCHMARKING_MSG(type) ADD_MSG(rosbag2_performance_benchmarking_msgs, type)
#define ADD_SENSOR_MSG(type) ADD_MSG(sensor_msgs, type)

namespace msg_utils
{
template<typename ... Args>
std::shared_ptr<ProducerBase> create(std::string key, Args & ... args)
{
  ADD_BENCHMARKING_MSG(ByteArray)

  ADD_SENSOR_MSG(BatteryState)
  ADD_SENSOR_MSG(CameraInfo)
  ADD_SENSOR_MSG(ChannelFloat32)
  ADD_SENSOR_MSG(CompressedImage)
  ADD_SENSOR_MSG(FluidPressure)
  ADD_SENSOR_MSG(Illuminance)
  ADD_SENSOR_MSG(Image)
  ADD_SENSOR_MSG(Imu)
  ADD_SENSOR_MSG(JointState)
  ADD_SENSOR_MSG(Joy)
  ADD_SENSOR_MSG(JoyFeedbackArray)
  ADD_SENSOR_MSG(LaserEcho)
  ADD_SENSOR_MSG(LaserScan)
  ADD_SENSOR_MSG(MagneticField)
  ADD_SENSOR_MSG(MultiDOFJointState)
  ADD_SENSOR_MSG(MultiEchoLaserScan)
  ADD_SENSOR_MSG(NavSatFix)
  ADD_SENSOR_MSG(PointCloud)
  ADD_SENSOR_MSG(PointCloud2)
  ADD_SENSOR_MSG(PointField)
  ADD_SENSOR_MSG(Range)
  ADD_SENSOR_MSG(RelativeHumidity)
  ADD_SENSOR_MSG(Temperature)
  ADD_SENSOR_MSG(TimeReference)

  throw std::runtime_error("Unknown message type: " + key);
}

}  // namespace msg_utils

#endif  // MSG_UTILS__MESSAGE_PRODUCER_FACTORY_HPP_
