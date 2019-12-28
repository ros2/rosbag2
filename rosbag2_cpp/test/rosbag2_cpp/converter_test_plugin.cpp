// Copyright 2018,  Open Source Robotics Foundation, Inc.
// Copyright 2018,  Bosch Software Innovations GmbH.
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

#include "converter_test_plugin.hpp"

#include <memory>

void ConverterTestPlugin::deserialize(
  const std::shared_ptr<const rosbag2_storage::SerializedBagMessage> serialized_message,
  const rosidl_message_type_support_t * type_support,
  std::shared_ptr<rosbag2_cpp::rosbag2_introspection_message_t> ros_message)
{
  (void) ros_message;
  (void) serialized_message;
  (void) type_support;
}

void ConverterTestPlugin::serialize(
  const std::shared_ptr<const rosbag2_cpp::rosbag2_introspection_message_t> ros_message,
  const rosidl_message_type_support_t * type_support,
  std::shared_ptr<rosbag2_storage::SerializedBagMessage> serialized_message)
{
  (void) serialized_message;
  (void) ros_message;
  (void) type_support;
}

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(ConverterTestPlugin,
  rosbag2_cpp::converter_interfaces::SerializationFormatConverter)
