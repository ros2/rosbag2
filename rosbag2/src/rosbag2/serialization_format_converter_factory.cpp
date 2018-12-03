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

#include "pluginlib/class_loader.hpp"
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
      std::make_unique<pluginlib::ClassLoader<SerializationFormatSerializerInterface>>(
      "rosbag2", "rosbag2::SerializationFormatSerializerInterface");
    deserializer_class_loader_ =
      std::make_unique<pluginlib::ClassLoader<SerializationFormatDeserializerInterface>>(
      "rosbag2", "rosbag2::SerializationFormatDeserializerInterface");
  } catch (const std::exception & e) {
    ROSBAG2_LOG_ERROR_STREAM("Unable to create class loader instance: " << e.what());
    throw e;
  }
}

SerializationFormatConverterFactory::~SerializationFormatConverterFactory() = default;

std::unique_ptr<SerializationFormatDeserializerInterface>
SerializationFormatConverterFactory::load_deserializer(const std::string & format)
{
  auto converter_id = format + "_converter";

  const auto & registered_converter_classes = converter_class_loader_->getDeclaredClasses();
  const auto & registered_deserializer_classes = deserializer_class_loader_->getDeclaredClasses();
  auto class_exists_in_converters = std::find(registered_converter_classes.begin(),
      registered_converter_classes.end(), converter_id);
  auto class_exists_in_deserializers = std::find(registered_deserializer_classes.begin(),
      registered_deserializer_classes.end(), converter_id);
  if (class_exists_in_converters == registered_converter_classes.end() &&
    class_exists_in_deserializers == registered_deserializer_classes.end())
  {
    ROSBAG2_LOG_ERROR_STREAM("Requested converter for format '" << format << "' does not exist");
    return nullptr;
  }

  try {
    return std::unique_ptr<SerializationFormatDeserializerInterface>(
      deserializer_class_loader_->createUnmanagedInstance(converter_id));
  } catch (const std::runtime_error & ex) {
    (void) ex;  // Ignore, try to load converter instead
  }

  try {
    return std::unique_ptr<SerializationFormatConverterInterface>(
      converter_class_loader_->createUnmanagedInstance(converter_id));
  } catch (const std::runtime_error & ex) {
    ROSBAG2_LOG_ERROR_STREAM("Unable to load instance of converter interface: " << ex.what());
    return nullptr;
  }
}

std::unique_ptr<SerializationFormatSerializerInterface>
SerializationFormatConverterFactory::load_serializer(const std::string & format)
{
  auto converter_id = format + "_converter";

  const auto & registered_converter_classes = converter_class_loader_->getDeclaredClasses();
  const auto & registered_serializer_classes = serializer_class_loader_->getDeclaredClasses();
  auto class_exists_in_converters = std::find(registered_converter_classes.begin(),
      registered_converter_classes.end(), converter_id);
  auto class_exists_in_serializers = std::find(registered_serializer_classes.begin(),
      registered_serializer_classes.end(), converter_id);
  if (class_exists_in_converters == registered_converter_classes.end() &&
    class_exists_in_serializers == registered_serializer_classes.end())
  {
    ROSBAG2_LOG_ERROR_STREAM("Requested converter for format '" << format << "' does not exist");
    return nullptr;
  }

  try {
    return std::unique_ptr<SerializationFormatSerializerInterface>(
      serializer_class_loader_->createUnmanagedInstance(converter_id));
  } catch (const std::runtime_error & ex) {
    (void) ex;  // Ignore, try to load converter instead
  }

  try {
    return std::unique_ptr<SerializationFormatConverterInterface>(
      converter_class_loader_->createUnmanagedInstance(converter_id));
  } catch (const std::runtime_error & ex) {
    ROSBAG2_LOG_ERROR_STREAM("Unable to load instance of converter interface: " << ex.what());
    return nullptr;
  }
}

}  // namespace rosbag2
