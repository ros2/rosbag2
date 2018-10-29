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

#include "rosbag2/converter.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rosbag2_storage/metadata_io.hpp"
#include "rosbag2/info.hpp"
#include "rosbag2/typesupport_helpers.hpp"
#include "rosbag2/storage_options.hpp"
#include "rosbag2_storage/ros_helper.hpp"

namespace rosbag2
{

struct ConverterTypeSupport
{
  const rosidl_message_type_support_t * type_support;
  const rosidl_message_type_support_t * introspection_type_support;
};

Converter::Converter(
  const std::string & input_format,
  const std::string & output_format,
  std::shared_ptr<rosbag2::SerializationFormatConverterFactoryInterface> converter_factory)
: converter_factory_(converter_factory)
{
  input_converter_ = converter_factory_->load_converter(input_format);
  output_converter_ = converter_factory_->load_converter(output_format);
  if (!input_converter_) {
    throw std::runtime_error("Could not find converter for format " + input_format);
  }
  if (!output_converter_) {
    throw std::runtime_error("Could not find converter for format " + output_format);
  }
}

Converter::~Converter()
{
  input_converter_.reset();
  output_converter_.reset();
  converter_factory_.reset();  // needs to be destroyed only after the converters
}

std::shared_ptr<SerializedBagMessage> Converter::convert(
  std::shared_ptr<const rosbag2::SerializedBagMessage> message)
{
  auto ts = topics_and_types_[message->topic_name].type_support;
  auto introspection_ts = topics_and_types_[message->topic_name].introspection_type_support;

  auto allocator = rcutils_get_default_allocator();
  std::shared_ptr<rosbag2_ros2_message_t> allocated_ros_message =
    allocate_ros2_message(introspection_ts, &allocator);

  input_converter_->deserialize(message, ts, allocated_ros_message);
  auto output_message = std::make_shared<rosbag2::SerializedBagMessage>();
  output_message->serialized_data = rosbag2_storage::make_empty_serialized_message(0);
  output_converter_->serialize(allocated_ros_message, ts, output_message);
  return output_message;
}

void Converter::add_topic(const std::string & topic, const std::string & type)
{
  ConverterTypeSupport type_support;
  type_support.type_support = get_typesupport(type, "rosidl_typesupport_cpp");
  type_support.introspection_type_support =
    get_typesupport(type, "rosidl_typesupport_introspection_cpp");

  topics_and_types_.insert({topic, type_support});
}

}  // namespace rosbag2
