// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#include "rosbag2_cpp/readers/simple_reader.hpp"

#include <memory>
#include <string>
#include <vector>

#include "rosbag2_cpp/serialization_format_converter_factory.hpp"
#include "rosbag2_cpp/types/introspection_message.hpp"
#include "rosbag2_cpp/typesupport_helpers.hpp"

#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"


namespace
{

/// Default serialization format is CDR
const char kSerializationFormat[] = "cdr";
/// Default storage format is sqlite
const char kStorageId[] = "sqlite3";
/// Load a max of one GB at a time
const size_t kMaxBagfileSize = 1e9;
/// Some string for creating introspection type support
const char kIntrospectionString[] = "rosidl_typesupport_introspection_cpp";
/// Some string for creating type support
const char kTypeSupportString[] = "rosidl_typesupport_cpp";

}  // namespace

namespace rosbag2_cpp
{
namespace readers
{

SimpleReader::SimpleReader()
: factory_(),
  deserializer_(factory_.load_deserializer(kSerializationFormat)),
  allocator_(rcutils_get_default_allocator()),
  current_message_(),
  reader_()
{
}

void SimpleReader::open(const std::string & filename)
{
  /// Initialize the storage options
  StorageOptions storage_options;

  storage_options.uri = filename;
  storage_options.storage_id = kStorageId;
  storage_options.max_bagfile_size = kMaxBagfileSize;

  /// Initialize the converter options
  ConverterOptions converter_options;

  converter_options.input_serialization_format = kSerializationFormat;
  converter_options.output_serialization_format = kSerializationFormat;

  /// Open the bag reader
  open(storage_options, converter_options);
}

void SimpleReader::open(
  const StorageOptions & storage_options,
  const ConverterOptions & converter_options)
{
  reader_.open(storage_options, converter_options);
}

void SimpleReader::reset()
{
  reader_.reset();
}

bool SimpleReader::has_next()
{
  return reader_.has_next();
}

std::shared_ptr<rosbag2_storage::SerializedBagMessage> SimpleReader::read_next()
{
  current_message_ = reader_.read_next();
  return current_message_;
}

std::vector<rosbag2_storage::TopicMetadata> SimpleReader::get_all_topics_and_types()
{
  return reader_.get_all_topics_and_types();
}

std::string SimpleReader::topic_type(const std::string & topic_name)
{
  /// \TODO: We can speed this up by caching the topics
  const std::vector<rosbag2_storage::TopicMetadata> kTopics = get_all_topics_and_types();
  for (const rosbag2_storage::TopicMetadata & topic : kTopics) {
    if (topic.name == topic_name) {
      return topic.type;
    }
  }

  return "";
}

std::shared_ptr<rosbag2_introspection_message_t> SimpleReader::deserializeMessage()
{
  /// Initialize a few messages
  const std::string kTopicName = current_message_->topic_name;
  const std::string kMessageType = topic_type(kTopicName);

  /// If we failed to find the type string then return
  if (kMessageType.empty()) {
    return nullptr;
  }

  /// Allocate space for message to be deserialized
  const rosidl_message_type_support_t * kSupport =
    get_typesupport(kMessageType, kIntrospectionString);
  std::shared_ptr<rosbag2_introspection_message_t> output_message(
    allocate_introspection_message(kSupport, &allocator_));
  output_message->time_stamp = 1;
  introspection_message_set_topic_name(output_message.get(), kTopicName.c_str());

  /// Get type support to deserialize the message
  const rosidl_message_type_support_t * kTypeSupport =
    get_typesupport(kMessageType, kTypeSupportString);

  /// Deserialize the message
  deserializer_->deserialize(current_message_, kTypeSupport, output_message);

  return output_message;
}

}  // namespace readers
}  // namespace rosbag2_cpp
