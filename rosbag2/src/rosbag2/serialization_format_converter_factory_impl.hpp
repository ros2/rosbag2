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

#ifndef ROSBAG2__SERIALIZATION_FORMAT_CONVERTER_FACTORY_IMPL_HPP_
#define ROSBAG2__SERIALIZATION_FORMAT_CONVERTER_FACTORY_IMPL_HPP_

#include "rosbag2/serialization_format_converter_factory_interface.hpp"

#include <memory>
#include <string>
#include <vector>

#include "pluginlib/class_loader.hpp"
#include "rosbag2/converter_interfaces/serialization_format_converter_interface.hpp"
#include "rosbag2/logging.hpp"
#include "rosbag2/visibility_control.hpp"

namespace rosbag2
{

const char * converter_suffix = "_converter";

class SerializationFormatConverterFactoryImpl
{
public:
  SerializationFormatConverterFactoryImpl()
  {
    try {
      converter_class_loader_ =
        std::make_unique<pluginlib::ClassLoader<
            converter_interfaces::SerializationFormatConverterInterface>>(
        "rosbag2", "rosbag2::converter_interfaces::SerializationFormatConverterInterface");
      serializer_class_loader_ =
        std::make_shared<pluginlib::ClassLoader<
            converter_interfaces::SerializationFormatSerializerInterface>>(
        "rosbag2", "rosbag2::converter_interfaces::SerializationFormatSerializerInterface");
      deserializer_class_loader_ =
        std::make_shared<pluginlib::ClassLoader<
            converter_interfaces::SerializationFormatDeserializerInterface>>(
        "rosbag2", "rosbag2::converter_interfaces::SerializationFormatDeserializerInterface");
    } catch (const std::exception & e) {
      ROSBAG2_LOG_ERROR_STREAM("Unable to create class loader instance: " << e.what());
      throw e;
    }
  }

  ~SerializationFormatConverterFactoryImpl() = default;

  std::unique_ptr<converter_interfaces::SerializationFormatDeserializerInterface>
  load_deserializer(const std::string & format)
  {
    return load_interface(format, deserializer_class_loader_);
  }

  std::unique_ptr<converter_interfaces::SerializationFormatSerializerInterface>
  load_serializer(const std::string & format)
  {
    return load_interface(format, serializer_class_loader_);
  }

private:
  bool is_plugin_registered(
    const std::string & converter_id,
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

  template<typename SerializationFormatIface>
  std::unique_ptr<SerializationFormatIface>
  load_interface(
    const std::string & format,
    std::shared_ptr<pluginlib::ClassLoader<SerializationFormatIface>> class_loader)
  {
    auto converter_id = format + converter_suffix;

    if (is_plugin_registered(
        converter_id,
        converter_class_loader_->getDeclaredClasses(),
        class_loader->getDeclaredClasses()))
    {
      ROSBAG2_LOG_ERROR_STREAM("Requested converter for format '" << format << "' does not exist");
      return nullptr;
    }

    try {
      return std::unique_ptr<SerializationFormatIface>(
        class_loader->createUnmanagedInstance(converter_id));
    } catch (const std::runtime_error & ex) {
      (void) ex;  // Ignore, try to load converter instead
    }

    try {
      return std::unique_ptr<converter_interfaces::SerializationFormatConverterInterface>(
        converter_class_loader_->createUnmanagedInstance(converter_id));
    } catch (const std::runtime_error & ex) {
      ROSBAG2_LOG_ERROR_STREAM("Unable to load instance of converter interface: " << ex.what());
      return nullptr;
    }
  }

  std::unique_ptr<
    pluginlib::ClassLoader<converter_interfaces::SerializationFormatConverterInterface>>
  converter_class_loader_;
  std::shared_ptr<
    pluginlib::ClassLoader<converter_interfaces::SerializationFormatSerializerInterface>>
  serializer_class_loader_;
  std::shared_ptr<
    pluginlib::ClassLoader<converter_interfaces::SerializationFormatDeserializerInterface>>
  deserializer_class_loader_;
};

}  // namespace rosbag2

#endif  // ROSBAG2__SERIALIZATION_FORMAT_CONVERTER_FACTORY_IMPL_HPP_
