// Copyright 2021 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#ifndef ROSBAG2_CPP__RMW_IMPLEMENTED_SERIALIZATION_FORMAT_CONVERTER_HPP_
#define ROSBAG2_CPP__RMW_IMPLEMENTED_SERIALIZATION_FORMAT_CONVERTER_HPP_

#include <memory>
#include <string>

#include "rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp"

namespace rosbag2_cpp
{
class RMWImplementedConverterImpl;

/**
 * Default implementation of the SerializationFormatConverter.
 *
 * This converter does not understand any serialization formats on its own, instead it
 * searches the system for an installed RMW implementation that understands the requested format,
 * loads that library if found, and uses its implementation for serialization.
 */
class RMWImplementedConverter
  : public rosbag2_cpp::converter_interfaces::SerializationFormatConverter
{
public:
  /**
   * Constructor.
   * \throws std::runtime_error if no RMW implementation was found supporting the format.
   */
  explicit RMWImplementedConverter(const std::string & format);
  virtual ~RMWImplementedConverter();

  void deserialize(
    std::shared_ptr<const rosbag2_storage::SerializedBagMessage> serialized_message,
    const rosidl_message_type_support_t * type_support,
    std::shared_ptr<rosbag2_cpp::rosbag2_introspection_message_t> introspection_message) override;

  void serialize(
    std::shared_ptr<const rosbag2_cpp::rosbag2_introspection_message_t> introspection_message,
    const rosidl_message_type_support_t * type_support,
    std::shared_ptr<rosbag2_storage::SerializedBagMessage> serialized_message) override;

private:
  std::unique_ptr<RMWImplementedConverterImpl> impl_;
};
}  // namespace rosbag2_cpp

#endif  // ROSBAG2_CPP__RMW_IMPLEMENTED_SERIALIZATION_FORMAT_CONVERTER_HPP_
