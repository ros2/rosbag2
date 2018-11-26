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

Converter::Converter(
  const std::string & input_format,
  const std::string & output_format,
  const std::vector<TopicMetadata> & topics_and_types,
  std::shared_ptr<rosbag2::SerializationFormatConverterFactoryInterface> converter_factory)
: converter_factory_(converter_factory)
{
  for (const auto & topic_with_type : topics_and_types) {
    topics_and_types_.insert({topic_with_type.name, topic_with_type.type});
  }
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
  auto ts = get_typesupport(topics_and_types_[message->topic_name], "rosidl_typesupport_cpp");
  auto introspection_ts =
    get_typesupport(topics_and_types_[message->topic_name], "rosidl_typesupport_introspection_cpp");
  auto allocator = rcutils_get_default_allocator();
  std::shared_ptr<rosbag2_ros2_message_t> allocated_ros_message =
    allocate_ros2_message(introspection_ts, &allocator);

  input_converter_->deserialize(message, ts, allocated_ros_message);
  auto output_message = std::make_shared<rosbag2::SerializedBagMessage>();
  output_message->serialized_data = rosbag2_storage::make_empty_serialized_message(0);
  output_converter_->serialize(allocated_ros_message, ts, output_message);
  return output_message;
}

}  // namespace rosbag2
