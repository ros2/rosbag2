// Copyright 2022, Foxglove Technologies. All rights reserved.
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

#ifndef ROSBAG2_CPP__MESSAGE_DEFINITIONS__LOCAL_MESSAGE_DEFINITION_SOURCE_HPP_
#define ROSBAG2_CPP__MESSAGE_DEFINITIONS__LOCAL_MESSAGE_DEFINITION_SOURCE_HPP_

#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "rosbag2_cpp/visibility_control.hpp"
#include "rosbag2_storage/message_definition.hpp"

namespace rosbag2_cpp
{

class DefinitionNotFoundError : public std::exception
{
private:
  std::string name_;

public:
  explicit DefinitionNotFoundError(std::string name)
  : name_(std::move(name))
  {
  }

  const char * what() const noexcept override
  {
    return name_.c_str();
  }
};

class LocalMessageDefinitionSource final
{
public:
  /**
   * Concatenate the message definition with its dependencies into a self-contained schema.
   * The format is different for MSG and IDL definitions, and is described fully in
   * docs/message_definition_encoding.md
   * Throws DefinitionNotFoundError if one or more definition files are missing for the given
   * package resource name.
   */
  rosbag2_storage::MessageDefinition get_full_text(const std::string & type_name);

  enum struct Format
  {
    UNKNOWN = 0,
    MSG = 1,
    IDL = 2,
  };

private:
  struct MessageSpec
  {
    MessageSpec(Format format, std::string text, const std::string & package_context);
    const std::set<std::string> dependencies;
    const std::string text;
    Format format{Format::UNKNOWN};
  };

  struct DefinitionIdentifier
  {
    DefinitionIdentifier() = delete;
    DefinitionIdentifier(std::string type_name, Format format)
    : type_name_(type_name)
      , format_(format)
    {
      size_t h1 = std::hash<Format>()(format_);
      size_t h2 = std::hash<std::string>()(type_name_);
      hash_ = h1 ^ h2;
    }
    bool operator==(const DefinitionIdentifier & di) const
    {
      return (format_ == di.format_) && (type_name_ == di.type_name_);
    }

    size_t hash() const
    {
      return hash_;
    }

    Format format() const
    {
      return format_;
    }

    std::string type_name() const
    {
      return type_name_;
    }

private:
    std::string type_name_;
    Format format_;
    size_t hash_;
  };

  struct DefinitionIdentifierHash
  {
    size_t operator()(const DefinitionIdentifier & di) const
    {
      return di.hash();
    }
  };

  /**
   * Load and parse the message file referenced by the given datatype, or return it from
   * msg_specs_by_datatype
   */
  const MessageSpec & load_message_spec(const DefinitionIdentifier & definition_identifier);

  static std::string delimiter(const DefinitionIdentifier & definition_identifier);

  std::unordered_map<DefinitionIdentifier,
    MessageSpec, DefinitionIdentifierHash> msg_specs_by_definition_identifier_;
};

ROSBAG2_CPP_PUBLIC
std::set<std::string> parse_definition_dependencies(
  LocalMessageDefinitionSource::Format format,
  const std::string & text,
  const std::string & package_context);

}  // namespace rosbag2_cpp

#endif  // ROSBAG2_CPP__MESSAGE_DEFINITIONS__LOCAL_MESSAGE_DEFINITION_SOURCE_HPP_
