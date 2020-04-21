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

#ifndef ROSBAG2_CPP__SERIALIZER_HPP_
#define ROSBAG2_CPP__SERIALIZER_HPP_

#include <memory>
#include <string>

#include "rcpputils/shared_library.hpp"

#include "rmw/serialized_message.h"

#include "rosbag2_cpp/converter_interfaces/serialization_format_deserializer.hpp"
#include "rosbag2_cpp/serialization_format_converter_factory.hpp"
#include "rosbag2_cpp/type_names.hpp"
#include "rosbag2_cpp/typesupport_helpers.hpp"
#include "rosbag2_cpp/visibility_control.hpp"


namespace rosbag2_cpp
{

/// \brief A simple class to serialize ros messages into SerializedBagMessages
class ROSBAG2_CPP_PUBLIC Serializer
{
public:
  explicit Serializer();
  ~Serializer() = default;

  /// \brief Serialize a ros messages into a SerializedBagMessage
  /// \param to_serialize The data message to be serialized
  /// \param topic_name Name of the topic to pack into the bag message
  template<class T>
  std::shared_ptr<rosbag2_storage::SerializedBagMessage> serialize(
    const T & to_serialize, const std::string & topic_name)
  {
    // Initialize the type string
    const std::string message_type = type_names<T>();

    // Get required type support
    const rosidl_message_type_support_t * support =
      rosbag2_cpp::get_typesupport(message_type, introspection_string_, introspection_library_);
    const rosidl_message_type_support_t * type_support =
      rosbag2_cpp::get_typesupport(message_type, type_support_string_, type_support_library_);

    // Initialize the message to serialize into
    std::shared_ptr<rosbag2_cpp::rosbag2_introspection_message_t> output_message =
      rosbag2_cpp::allocate_introspection_message(support, &allocator_);
    output_message->time_stamp = 1;
    rosbag2_cpp::introspection_message_set_topic_name(output_message.get(), topic_name.c_str());

    // Fill in the message contents
    T * correctlyTypedMessage = reinterpret_cast<T *>(output_message->message);
    *correctlyTypedMessage = to_serialize;

    // Copy the data into the actual bag message object
    std::shared_ptr<rosbag2_storage::SerializedBagMessage> bag_message =
      std::make_shared<rosbag2_storage::SerializedBagMessage>();
    bag_message->serialized_data =
      std::shared_ptr<rmw_serialized_message_t>(new rmw_serialized_message_t());
    auto ret = rmw_serialized_message_init(bag_message->serialized_data.get(), 0, &allocator_);
    if (ret != RCUTILS_RET_OK) {
      return nullptr;
    }

    // Serialize the message
    serializer_->serialize(output_message, type_support, bag_message);

    return bag_message;
  }

private:
  /// Serialization factory. This seems to need to stay in scope
  rosbag2_cpp::SerializationFormatConverterFactory factory_;
  /// Serializer object
  std::unique_ptr<rosbag2_cpp::converter_interfaces::SerializationFormatSerializer> serializer_;
  /// A generic allocator for deserialization
  const rcutils_allocator_t allocator_;
  /// Shared library for introspection type support
  std::shared_ptr<rcpputils::SharedLibrary> introspection_library_;
  /// Shared library for type support
  std::shared_ptr<rcpputils::SharedLibrary> type_support_library_;
  /// Introspection type support string
  const std::string type_support_string_;
  /// Type support string
  const std::string introspection_string_;
};

}  // namespace rosbag2_cpp

#endif  // ROSBAG2_CPP__SERIALIZER_HPP_
