// Copyright 2018, Bosch Software Innovations GmbH.
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

#ifndef ROSBAG2_COMPRESSION__MOCK_CONVERTER_FACTORY_HPP_
#define ROSBAG2_COMPRESSION__MOCK_CONVERTER_FACTORY_HPP_

#include <gmock/gmock.h>

#include <memory>
#include <string>

#include "rosbag2_cpp/serialization_format_converter_factory_interface.hpp"

class MockConverterFactory : public rosbag2_cpp::SerializationFormatConverterFactoryInterface
{
public:
  MOCK_METHOD1(
    load_serializer,
    std::unique_ptr<rosbag2_cpp::converter_interfaces::SerializationFormatSerializer>(
      const std::string &));

  MOCK_METHOD1(
    load_deserializer,
    std::unique_ptr<rosbag2_cpp::converter_interfaces::SerializationFormatDeserializer>(
      const std::string &));
};

#endif  // ROSBAG2_COMPRESSION__MOCK_CONVERTER_FACTORY_HPP_
