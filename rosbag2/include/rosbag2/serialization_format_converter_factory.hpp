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

#ifndef ROSBAG2__SERIALIZATION_FORMAT_CONVERTER_FACTORY_HPP_
#define ROSBAG2__SERIALIZATION_FORMAT_CONVERTER_FACTORY_HPP_

#include "rosbag2/serialization_format_converter_factory_interface.hpp"

#include <memory>
#include <string>
#include <vector>

#include "pluginlib/class_loader.hpp"
#include "rosbag2/converter_interfaces/serialization_format_converter_interface.hpp"
#include "rosbag2/logging.hpp"
#include "rosbag2/visibility_control.hpp"

// This is necessary because of using stl types here. It is completely safe, because
// a) the member is not accessible from the outside
// b) there are no inline functions.
#ifdef _WIN32
# pragma warning(push)
# pragma warning(disable:4251)
#endif

namespace rosbag2
{

class ROSBAG2_PUBLIC SerializationFormatConverterFactory
  : public SerializationFormatConverterFactoryInterface
{
public:
  SerializationFormatConverterFactory();

  ~SerializationFormatConverterFactory() override;

  std::unique_ptr<converter_interfaces::SerializationFormatDeserializerInterface>
  load_deserializer(const std::string & format) override;

  std::unique_ptr<converter_interfaces::SerializationFormatSerializerInterface>
  load_serializer(const std::string & format) override;

private:
  bool check_that_plugin_is_registered(
    std::string converter_id,
    const std::vector<std::string> & registered_converter_classes,
    const std::vector<std::string> & registered_interface_classes);

  template<typename SerializationFormatIface>
  std::unique_ptr<SerializationFormatIface>
  load_interface(
    const std::string & format,
    std::shared_ptr<pluginlib::ClassLoader<SerializationFormatIface>> class_loader)
  {
    auto converter_id = format + "_converter";

    if (check_that_plugin_is_registered(
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

#ifdef _WIN32
# pragma warning(pop)
#endif

#endif  // ROSBAG2__SERIALIZATION_FORMAT_CONVERTER_FACTORY_HPP_
