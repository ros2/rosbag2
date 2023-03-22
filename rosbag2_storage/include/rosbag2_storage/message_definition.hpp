// Copyright 2023, Foxglove Technologies.
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

#ifndef ROSBAG2_STORAGE__MESSAGE_DEFINITION_HPP_
#define ROSBAG2_STORAGE__MESSAGE_DEFINITION_HPP_

#include <string>
#include <cassert>

#include "rosbag2_storage/visibility_control.hpp"

namespace rosbag2_storage
{

struct MessageDefinition
{
  /// @brief  The name of the type.
  ///
  /// Should match the `name` in TopicMetadata for all topics using this message definition.
  std::string name;
  /// @brief  The type description hash of the type described by this MessageDefinition.
  ///
  /// Should match the `type_hash` in TopicMetadata for all topics using this message definition.
  std::string type_hash;
  /// @brief The full encoded message definition for this type.
  std::string encoded_message_definition;

  enum class Encoding : uint8_t
  {
    /// \brief Default value
    Unknown = 0,
    /// @brief concatenated `.msg` files for this message definition and all dependent types.
    ConcatenatedMsg,
    /// @brief concatenated `.idl` files for this message definition and all dependent types.
    ConcatenatedIdl,
  };
  /// @brief The encoding technique used in `encoded_message_definition`.
  ///
  /// See docs/message_definition_encoding.md for details of each encoding.
  Encoding encoding{Encoding::Unknown};

  bool operator==(const MessageDefinition & rhs) const
  {
    return rhs.encoding == encoding && rhs.name == name && rhs.type_hash == type_hash &&
           rhs.encoded_message_definition == encoded_message_definition;
  }
  /// @brief returns the name of the encoding technique as a string.
  std::string encoding_name() const
  {
    switch (encoding) {
      case Encoding::ConcatenatedIdl:
        return "ros2idl";
      case Encoding::ConcatenatedMsg:
        return "ros2msg";
      default:
        return "unknown";
    }
  }

  /// @brief returns the Encoding enum value corresponding to the encoding string
  Encoding encoding_from_string(const std::string & encoding_str) const
  {
    Encoding encoding_value{Encoding::ConcatenatedMsg};
    if (encoding_str == "ros2msg") {
      encoding_value = Encoding::ConcatenatedMsg;
    } else if (encoding_str == "ros2idl") {
      encoding_value = Encoding::ConcatenatedIdl;
    } else {
      encoding_value = Encoding::Unknown;
    }
    return encoding_value;
  }
};

}  // namespace rosbag2_storage

#endif  // ROSBAG2_STORAGE__MESSAGE_DEFINITION_HPP_
