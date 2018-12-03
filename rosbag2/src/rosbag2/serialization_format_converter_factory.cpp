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

#include "rosbag2/serialization_format_converter_factory.hpp"

#include <memory>
#include <string>
#include <vector>

#include "rosbag2/logging.hpp"

namespace rosbag2
{

SerializationFormatConverterFactory::SerializationFormatConverterFactory()
{
  try {
    converter_class_loader_ =
      std::make_unique<pluginlib::ClassLoader<SerializationFormatConverterInterface>>(
      "rosbag2", "rosbag2::SerializationFormatConverterInterface");
    serializer_class_loader_ =
      std::make_shared<pluginlib::ClassLoader<SerializationFormatSerializerInterface>>(
      "rosbag2", "rosbag2::SerializationFormatSerializerInterface");
    deserializer_class_loader_ =
      std::make_shared<pluginlib::ClassLoader<SerializationFormatDeserializerInterface>>(
      "rosbag2", "rosbag2::SerializationFormatDeserializerInterface");
  } catch (const std::exception & e) {
    ROSBAG2_LOG_ERROR_STREAM("Unable to create class loader instance: " << e.what());
    throw e;
  }
}

SerializationFormatConverterFactory::~SerializationFormatConverterFactory() = default;

bool SerializationFormatConverterFactory::check_that_plugin_is_registered(
  std::string converter_id,
  const std::vector<std::string> & registered_converter_classes,
  const std::vector<std::string> & registered_interface_classes)
{
  auto class_exists_in_converters = std::find(registered_converter_classes.begin(),
      registered_converter_classes.end(), converter_id);
  auto class_exists_in_deserializers = std::find(registered_interface_classes.begin(),
      registered_interface_classes.end(), converter_id);
  return class_exists_in_converters == registered_converter_classes.end() &&
         class_exists_in_deserializers == registered_interface_classes.end();
}

std::unique_ptr<SerializationFormatDeserializerInterface>
SerializationFormatConverterFactory::load_deserializer(const std::string & format)
{
  return load_interface(format, deserializer_class_loader_);
}

std::unique_ptr<SerializationFormatSerializerInterface>
SerializationFormatConverterFactory::load_serializer(const std::string & format)
{
  return load_interface(format, serializer_class_loader_);
}

}  // namespace rosbag2
