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

#include "rosbag2_cpp/message_definitions/message_definition_cache.hpp"

#include <rcutils/logging_macros.h>

#include <fstream>
#include <functional>
#include <optional>
#include <regex>
#include <set>
#include <string>
#include <unordered_set>
#include <utility>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ament_index_cpp/get_resource.hpp>
#include <ament_index_cpp/get_resources.hpp>

namespace rosbag2_cpp
{
// Match datatype names (foo_msgs/Bar or foo_msgs/msg/Bar)
static const std::regex PACKAGE_TYPENAME_REGEX{R"(^([a-zA-Z0-9_]+)/(?:msg/)?([a-zA-Z0-9_]+)$)"};

// Match field types from .msg definitions ("foo_msgs/Bar" in "foo_msgs/Bar[] bar")
static const std::regex MSG_FIELD_TYPE_REGEX{R"((?:^|\n)\s*([a-zA-Z0-9_/]+)(?:\[[^\]]*\])?\s+)"};

// match field types from `.idl` definitions ("foo_msgs/msg/bar" in #include <foo_msgs/msg/Bar.idl>)
static const std::regex IDL_FIELD_TYPE_REGEX{
  R"((?:^|\n)#include\s+(?:"|<)([a-zA-Z0-9_/]+)\.idl(?:"|>))"};

static const std::unordered_set<std::string> PRIMITIVE_TYPES{
  "bool", "byte", "char", "float32", "float64", "int8", "uint8",
  "int16", "uint16", "int32", "uint32", "int64", "uint64", "string"};

static std::set<std::string> parse_msg_dependencies(
  const std::string & text,
  const std::string & package_context)
{
  std::set<std::string> dependencies;

  for (std::sregex_iterator iter(text.begin(), text.end(), MSG_FIELD_TYPE_REGEX);
    iter != std::sregex_iterator(); ++iter)
  {
    std::string type = (*iter)[1];
    if (PRIMITIVE_TYPES.find(type) != PRIMITIVE_TYPES.end()) {
      continue;
    }
    if (type.find('/') == std::string::npos) {
      dependencies.insert(package_context + '/' + std::move(type));
    } else {
      dependencies.insert(std::move(type));
    }
  }
  return dependencies;
}

static std::set<std::string> parse_idl_dependencies(const std::string & text)
{
  std::set<std::string> dependencies;

  for (std::sregex_iterator iter(text.begin(), text.end(), IDL_FIELD_TYPE_REGEX);
    iter != std::sregex_iterator(); ++iter)
  {
    dependencies.insert((*iter)[1]);
  }
  return dependencies;
}

std::set<std::string> parse_definition_dependencies(
  MessageDefinitionCache::Format format,
  const std::string & text,
  const std::string & package_context)
{
  switch (format) {
    case MessageDefinitionCache::Format::MSG:
      return parse_msg_dependencies(text, package_context);
    case MessageDefinitionCache::Format::IDL:
      return parse_idl_dependencies(text);
    default:
      throw std::runtime_error("switch is not exhaustive");
  }
}

static const char * extension_for_format(MessageDefinitionCache::Format format)
{
  switch (format) {
    case MessageDefinitionCache::Format::MSG:
      return ".msg";
    case MessageDefinitionCache::Format::IDL:
      return ".idl";
    default:
      throw std::runtime_error("switch is not exhaustive");
  }
}

std::string MessageDefinitionCache::delimiter(const DefinitionIdentifier & definition_identifier)
{
  std::string result =
    "================================================================================\n";
  switch (definition_identifier.format) {
    case Format::MSG:
      result += "MSG: ";
      break;
    case Format::IDL:
      result += "IDL: ";
      break;
    default:
      throw std::runtime_error("switch is not exhaustive");
  }
  result += definition_identifier.package_resource_name;
  result += "\n";
  return result;
}

MessageDefinitionCache::MessageSpec::MessageSpec(
  Format format, std::string text,
  const std::string & package_context)
: dependencies(parse_definition_dependencies(format, text, package_context))
  , text(std::move(text))
  , format(format)
{
}

const MessageDefinitionCache::MessageSpec & MessageDefinitionCache::load_message_spec(
  const DefinitionIdentifier & definition_identifier)
{
  if (auto it = msg_specs_by_definition_identifier_.find(definition_identifier);
    it != msg_specs_by_definition_identifier_.end())
  {
    return it->second;
  }
  std::smatch match;
  if (!std::regex_match(
      definition_identifier.package_resource_name, match,
      PACKAGE_TYPENAME_REGEX))
  {
    throw std::invalid_argument(
            "Invalid package resource name: " +
            definition_identifier.package_resource_name);
  }
  std::string package = match[1];
  std::string share_dir = ament_index_cpp::get_package_share_directory(package);
  std::ifstream file{share_dir + "/msg/" + match[2].str() +
    extension_for_format(definition_identifier.format)};
  if (!file.good()) {
    throw DefinitionNotFoundError(definition_identifier.package_resource_name);
  }

  std::string contents{std::istreambuf_iterator(file), {}};
  const MessageSpec & spec =
    msg_specs_by_definition_identifier_
    .emplace(
    definition_identifier,
    MessageSpec(definition_identifier.format, std::move(contents), package))
    .first->second;

  // "References and pointers to data stored in the container are only invalidated by erasing that
  // element, even when the corresponding iterator is invalidated."
  return spec;
}

rosbag2_storage::MessageDefinition MessageDefinitionCache::get_full_text(
  const std::string & root_package_resource_name)
{
  std::unordered_set<DefinitionIdentifier, DefinitionIdentifierHash> seen_deps;

  std::function<std::string(const DefinitionIdentifier &)> append_recursive =
    [&](const DefinitionIdentifier & definition_identifier) {
      const MessageSpec & spec = load_message_spec(definition_identifier);
      std::string result = spec.text;
      for (const auto & dep_name : spec.dependencies) {
        DefinitionIdentifier dep{definition_identifier.format, dep_name};
        bool inserted = seen_deps.insert(dep).second;
        if (inserted) {
          result += "\n";
          result += delimiter(dep);
          result += append_recursive(dep);
        }
      }
      return result;
    };

  std::string result;
  Format format = Format::MSG;
  try {
    result = append_recursive(DefinitionIdentifier{format, root_package_resource_name});
  } catch (const DefinitionNotFoundError & err) {
    // log that we've fallen back
    RCUTILS_LOG_WARN_NAMED(
      "rosbag2_cpp", "no .msg definition for %s, falling back to IDL",
      err.what());
    format = Format::IDL;
    DefinitionIdentifier root_definition_identifier{format, root_package_resource_name};
    result = delimiter(root_definition_identifier) + append_recursive(root_definition_identifier);
  }
  rosbag2_storage::MessageDefinition out;
  switch (format) {
    case Format::MSG:
      out.encoding = "ros2msg";
      break;
    case Format::IDL:
      out.encoding = "ros2idl";
      break;
  }
  out.encoded_message_definition = result;
  out.type_name = root_package_resource_name;
  return out;
}
}  // namespace rosbag2_cpp
