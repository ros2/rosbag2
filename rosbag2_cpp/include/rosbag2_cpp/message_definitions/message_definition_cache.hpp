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

#ifndef ROSBAG2_CPP__MESSAGE_DEFINITIONS__MESSAGE_DEFINITION_CACHE_HPP_
#define ROSBAG2_CPP__MESSAGE_DEFINITIONS__MESSAGE_DEFINITION_CACHE_HPP_

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

class MessageDefinitionCache final
{
public:
  /**
   * Concatenate the message definition with its dependencies into a self-contained schema.
   * The format is different for MSG and IDL definitions, and is described fully in
   * docs/message_definition_encoding.md
   * Throws DefinitionNotFoundError if one or more definition files are missing for the given
   * package resource name.
   */
  rosbag2_storage::MessageDefinition get_full_text(const std::string & package_resource_name);

  enum struct Format
  {
    MSG,
    IDL,
  };

private:
  struct MessageSpec
  {
    MessageSpec(Format format, std::string text, const std::string & package_context);
    const std::set<std::string> dependencies;
    const std::string text;
    Format format;
  };

  struct DefinitionIdentifier
  {
    Format format;
    std::string package_resource_name;

    bool operator==(const DefinitionIdentifier & di) const
    {
      return (format == di.format) && (package_resource_name == di.package_resource_name);
    }
  };

  struct DefinitionIdentifierHash
  {
    std::size_t operator()(const DefinitionIdentifier & di) const
    {
      std::size_t h1 = std::hash<Format>()(di.format);
      std::size_t h2 = std::hash<std::string>()(di.package_resource_name);
      return h1 ^ h2;
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
  MessageDefinitionCache::Format format,
  const std::string & text,
  const std::string & package_context);

}  // namespace rosbag2_cpp

#endif  // ROSBAG2_CPP__MESSAGE_DEFINITIONS__MESSAGE_DEFINITION_CACHE_HPP_
