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

#include "rosbag2/serialization_format_converter_interface.hpp"
#include "rosbag2/visibility_control.hpp"

// This is necessary because of using stl types here. It is completely safe, because
// a) the member is not accessible from the outside
// b) there are no inline functions.
#ifdef _WIN32
# pragma warning(push)
# pragma warning(disable:4251)
#endif

namespace pluginlib
{

template<class T>
class ClassLoader;

}  // namespace pluginlib

namespace rosbag2
{

class ROSBAG2_PUBLIC SerializationFormatConverterFactory
  : public SerializationFormatConverterFactoryInterface
{
public:
  SerializationFormatConverterFactory();

  ~SerializationFormatConverterFactory() override;

  std::shared_ptr<SerializationFormatConverterInterface>
  load_converter(const std::string & format) override;

private:
  std::unique_ptr<pluginlib::ClassLoader<SerializationFormatConverterInterface>> class_loader_;
};

}  // namespace rosbag2

#ifdef _WIN32
# pragma warning(pop)
#endif

#endif  // ROSBAG2__SERIALIZATION_FORMAT_CONVERTER_FACTORY_HPP_
