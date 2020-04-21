// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#ifndef ROSBAG2_CPP__DESERIALIZER_HPP_
#define ROSBAG2_CPP__DESERIALIZER_HPP_

#include <memory>
#include <string>

#include "rcpputils/shared_library.hpp"

#include "rosbag2_cpp/converter_interfaces/serialization_format_deserializer.hpp"
#include "rosbag2_cpp/serialization_format_converter_factory.hpp"
#include "rosbag2_cpp/type_names.hpp"
#include "rosbag2_cpp/types/introspection_message.hpp"
#include "rosbag2_cpp/typesupport_helpers.hpp"
#include "rosbag2_cpp/visibility_control.hpp"


namespace rosbag2_cpp
{

class ROSBAG2_CPP_PUBLIC Deserializer
{
public:
  explicit Deserializer();
  ~Deserializer() = default;

  /// \brief Simple method to extract message data from a SerializedBagMessage
  /// \param message Bag message with the data to be extracted
  template<class T>
  T deserialize(const std::shared_ptr<rosbag2_storage::SerializedBagMessage> message)
  {
    // Initialize a few strings
    const std::string topic_name = message->topic_name;
    const std::string message_type = type_names<T>();

    // Get required type support
    const rosidl_message_type_support_t * support =
      rosbag2_cpp::get_typesupport(message_type, introspection_string_, introspection_library_);
    const rosidl_message_type_support_t * typeSupport =
      rosbag2_cpp::get_typesupport(message_type, type_support_string_, type_support_library_);

    // Initialize a message to deserialize into
    std::shared_ptr<rosbag2_cpp::rosbag2_introspection_message_t> output_message(
      rosbag2_cpp::allocate_introspection_message(support, &allocator_));
    output_message->time_stamp = 1;
    rosbag2_cpp::introspection_message_set_topic_name(output_message.get(), topic_name.c_str());

    // Deserialize the message
    deserializer_->deserialize(message, typeSupport, output_message);

    return *(static_cast<T *>(output_message->message));
  }

private:
  /// Serialization factory. This seems to need to stay in scope
  rosbag2_cpp::SerializationFormatConverterFactory factory_;
  /// Deserializer object
  std::unique_ptr<rosbag2_cpp::converter_interfaces::SerializationFormatDeserializer> deserializer_;
  /// A generic allocator for deserialization
  const rcutils_allocator_t allocator_;
  /// Shared library for introspection type support
  std::shared_ptr<rcpputils::SharedLibrary> introspection_library_;
  /// Shared library for type support
  std::shared_ptr<rcpputils::SharedLibrary> type_support_library_;
  /// Introspection type support string
  const std::string introspection_string_;
  /// Type support string
  const std::string type_support_string_;
};

}  // namespace rosbag2_cpp

#endif  // ROSBAG2_CPP__DESERIALIZER_HPP_
