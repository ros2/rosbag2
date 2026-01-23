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

#include <filesystem>
#include <fstream>
#include <functional>
#include <optional>
#include <regex>
#include <set>
#include <string>
#include <unordered_set>
#include <utility>

#include <ament_index_cpp/get_package_prefix.hpp>
#include "ament_index_cpp/get_resource.hpp"

#include "rosbag2_cpp/action_utils.hpp"
#include "rosbag2_cpp/service_utils.hpp"
#include "rosbag2_cpp/logging.hpp"
#include "rosbag2_cpp/message_definitions/local_message_definition_source.hpp"

namespace rosbag2_cpp
{

namespace fs = std::filesystem;

/// A type name did not match expectations, so a definition could not be looked for.
class TypenameNotUnderstoodError : public std::exception
{
private:
  std::string name_;

public:
  explicit TypenameNotUnderstoodError(std::string name)
  : name_(std::move(name))
  {}

  const char * what() const noexcept override
  {
    return name_.c_str();
  }
};

/// \brief This regular expression is designed to parse ROS topic type strings into their package
/// name and type name components.
/// \details  The regex requires a format like package_name/[path/]TypeName or
///  package_name/[path/][msg|srv|action]/TypeName where both package_name and TypeName consist of
///  alphanumeric characters and underscores.
/// Breaking down this regex:
/// ^ - Ensures matching starts at beginning of string
/// ([a-zA-Z0-9_]+) - First capture group: matches the package name
/// (e.g., "std_msgs", "sensor_msgs")
/// (?:/[a-zA-Z0-9_]+)* - Allows for any number of additional path segments (like /nested_sub_dir)
/// / - Matches a literal forward slash
/// (?:msg/|srv/|action/)? - Non-capturing group that optionally matches "msg/", "srv/", or
/// "action/"
/// ([a-zA-Z0-9_]+) - Second capture group: matches the type name (e.g., "String", "LaserScan")
/// $ - Ensures matching ends at the end of string
/// The regex processes ROS topic types in these formats:
/// package_name/TypeName (e.g., "std_msgs/String")
/// package_name/msg/TypeName (e.g., "std_msgs/msg/String")
/// package_name/srv/TypeName (e.g., "std_srvs/srv/SetBool")
/// package_name/action/TypeName (e.g., "nav2_msgs/action/NavigateToPose")
/// package_name/nested_sub_dir/action/TypeName
/// (e.g., "rosbag2_test_msgdefs/nested_sub_dir/action/BasicMsg")
/// package_name/msg/nested_sub_dir/TypeName
/// (e.g., "rosbag2_test_msgdefs/msg/nested_sub_dir/AnotherBasicMsg")
/// \note Invalid topic type formats, such as those with
///  std-msgs/String - hyphens are not allowed in package names
///  std_msgs/String.msg - file extensions are not allowed
///  std msgs/String - spaces are not allowed
///  std_msgs/@String - special characters other than alphanumeric and underscore are not allowed
static const std::regex PACKAGE_TYPENAME_REGEX{
  R"(^([a-zA-Z0-9_]+)(?:/[a-zA-Z0-9_]+)*/(?:msg/|srv/|action/)?([a-zA-Z0-9_]+)$)"};

// Match field types from .msg, .srv and .action definitions
// ("foo_msgs/Bar" in "foo_msgs/Bar[] bar")
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
  LocalMessageDefinitionSource::Format format,
  const std::string & text,
  const std::string & package_context)
{
  switch (format) {
    case LocalMessageDefinitionSource::Format::MSG:
      return parse_msg_dependencies(text, package_context);
    case LocalMessageDefinitionSource::Format::IDL:
      return parse_idl_dependencies(text);
    case LocalMessageDefinitionSource::Format::SRV:
    case LocalMessageDefinitionSource::Format::ACTION:
      {
        auto dep = parse_msg_dependencies(text, package_context);
        if (!dep.empty()) {
          return dep;
        } else {
          return parse_idl_dependencies(text);
        }
      }
    default:
      throw std::runtime_error("switch is not exhaustive");
  }
}

static const char * extension_for_format(LocalMessageDefinitionSource::Format format)
{
  switch (format) {
    case LocalMessageDefinitionSource::Format::MSG:
      return ".msg";
    case LocalMessageDefinitionSource::Format::IDL:
      return ".idl";
    case LocalMessageDefinitionSource::Format::SRV:
      return ".srv";
    case LocalMessageDefinitionSource::Format::ACTION:
      return ".action";
    default:
      throw std::runtime_error("switch is not exhaustive");
  }
}

std::string LocalMessageDefinitionSource::delimiter(
  const DefinitionIdentifier & definition_identifier)
{
  std::string result =
    "================================================================================\n";
  switch (definition_identifier.format()) {
    case Format::MSG:
      result += "MSG: ";
      break;
    case Format::IDL:
      result += "IDL: ";
      break;
    case Format::SRV:
      result += "SRV: ";
      break;
    case Format::ACTION:
      result += "ACTION: ";
      break;
    default:
      throw std::runtime_error("switch is not exhaustive");
  }
  result += definition_identifier.topic_type();
  result += "\n";
  return result;
}

LocalMessageDefinitionSource::MessageSpec::MessageSpec(
  Format format, std::string text,
  const std::string & package_context)
: dependencies(parse_definition_dependencies(format, text, package_context))
  , text(std::move(text))
  , format(format)
{
}

const LocalMessageDefinitionSource::MessageSpec & LocalMessageDefinitionSource::load_message_spec(
  const DefinitionIdentifier & definition_identifier)
{
  if (auto it = msg_specs_by_definition_identifier_.find(definition_identifier);
    it != msg_specs_by_definition_identifier_.end())
  {
    return it->second;
  }
  std::smatch match;
  const std::string topic_type = definition_identifier.topic_type();
  if (!std::regex_match(topic_type, match, PACKAGE_TYPENAME_REGEX) || match.size() < 3) {
    throw TypenameNotUnderstoodError(topic_type);
  }
  std::string package_name = match[1].str();
  const std::string file_name =
    match[2].str() + extension_for_format(definition_identifier.format());
  fs::path share_dir_path;
  // Get the resource content and prefix path from ament_index
  auto result = ament_index_cpp::get_resource("rosidl_interfaces", package_name);
  if (result.resourcePath != std::nullopt) {
    share_dir_path = result.resourcePath.value() / "share" / package_name;
    ROSBAG2_CPP_LOG_DEBUG(
      "resource_content : \n%s for package: '%s' ,\n share_dir: '%s'\n, topic_type: '%s'",
      result.contents.c_str(), package_name.c_str(), share_dir_path.c_str(), topic_type.c_str());
  } else {
    ROSBAG2_CPP_LOG_DEBUG(
      "Failed to get information about rosidl_interfaces resources from ament_index for package "
      "'%s'", package_name.c_str());
    throw DefinitionNotFoundError(definition_identifier.topic_type());
  }

  // Parse the resource content to find the relative file path matching the file name.
  std::string relative_file_path_str;
  std::stringstream ss(result.contents);
  std::string line;
  while (std::getline(ss, line, '\n')) {
    if (!line.empty()) {
      fs::path curr_relative_file_path(line);
      // Find the first line that ends with the filename we're looking for
      if (curr_relative_file_path.filename() == file_name) {
        relative_file_path_str = curr_relative_file_path.generic_string();
        break;
      }
    }
  }

  if (relative_file_path_str.empty()) {
    ROSBAG2_CPP_LOG_DEBUG(
      "Message definition file '%s' not found in the resource content for package: '%s'",
      file_name.c_str(), package_name.c_str());
    throw DefinitionNotFoundError(definition_identifier.topic_type());
  }
  std::string msg_definition_path_str = (share_dir_path / relative_file_path_str).generic_string();
  std::ifstream file{msg_definition_path_str};
  if (!file.good()) {
    ROSBAG2_CPP_LOG_DEBUG("Message definition not found in the %s for package: '%s'",
      msg_definition_path_str.c_str(), package_name.c_str());
    throw DefinitionNotFoundError(definition_identifier.topic_type());
  }
  ROSBAG2_CPP_LOG_DEBUG("Message definition found in the %s for package: '%s'",
      msg_definition_path_str.c_str(), package_name.c_str());

  std::string contents{std::istreambuf_iterator(file), {}};
  const MessageSpec & spec = msg_specs_by_definition_identifier_.emplace(
    definition_identifier,
    MessageSpec(definition_identifier.format(), std::move(contents), package_name)).first->second;

  // "References and pointers to data stored in the container are only invalidated by erasing that
  // element, even when the corresponding iterator is invalidated."
  return spec;
}

rosbag2_storage::MessageDefinition LocalMessageDefinitionSource::get_full_text(
  const std::string & root_type)
{
  return get_full_text_ext(root_type, std::string{});
}

rosbag2_storage::MessageDefinition LocalMessageDefinitionSource::get_full_text_ext(
  const std::string & root_type,
  const std::string & topic_name)
{
  std::unordered_set<DefinitionIdentifier, DefinitionIdentifierHash> seen_deps;

  std::function<std::string(const DefinitionIdentifier &, int32_t)> append_recursive =
    [&](const DefinitionIdentifier & definition_identifier, int32_t depth) {
      if (depth <= 0) {
        throw std::runtime_error{
                "Reached max recursion depth resolving definition of " + root_type};
      }
      const MessageSpec & spec = load_message_spec(definition_identifier);
      std::string result = spec.text;
      for (const auto & dep_name : spec.dependencies) {
        DefinitionIdentifier dep(dep_name, definition_identifier.format());
        bool inserted = seen_deps.insert(dep).second;
        if (inserted) {
          result += "\n";
          result += delimiter(dep);
          result += append_recursive(dep, depth - 1);
        }
      }
      return result;
    };

  std::string result;
  Format format = Format::UNKNOWN;
  int32_t max_recursion_depth = ROSBAG2_CPP_LOCAL_MESSAGE_DEFINITION_SOURCE_MAX_RECURSION_DEPTH;

  bool is_action_type =
    root_type.find("/action/") != std::string::npos ||
    root_type == "action_msgs/msg/GoalStatusArray" ||
    root_type == "action_msgs/srv/CancelGoal_Event";
  bool is_service_type = (!is_action_type && root_type.find("/srv/") != std::string::npos);

  // Note: If the root_type is a service event type or one of the action interface types, we will
  // try to convert it to the original service or action type respectively.
  std::string real_root_type = root_type;

  if (!is_service_type && !is_action_type) {  // Only for regular message types
    try {
      format = Format::MSG;
      // Note: By design The top-level message definition for MSG format is present first, with no
      // delimiter. All dependent .msg definitions are preceded by a two-line delimiter:
      result = append_recursive(DefinitionIdentifier(root_type, format), max_recursion_depth);
    } catch (const DefinitionNotFoundError & err) {
      ROSBAG2_CPP_LOG_DEBUG("No .msg definition for %s, falling back to IDL", err.what());
      format = Format::IDL;
      try {
        DefinitionIdentifier root_definition_identifier(root_type, format);
        result = (delimiter(root_definition_identifier) +
          append_recursive(root_definition_identifier, max_recursion_depth));
      } catch (const DefinitionNotFoundError & idl_search_error) {
        ROSBAG2_CPP_LOG_DEBUG("No .idl definition found for topic type %s.",
                              idl_search_error.what());
        format = Format::UNKNOWN;
      }
    } catch (const TypenameNotUnderstoodError & err) {
      ROSBAG2_CPP_LOG_DEBUG(
        "Message type name '%s' not understood by type definition search.", err.what());
      format = Format::UNKNOWN;
    }
  } else {
    // The service and action dependencies could be either in the msg or idl files.
    // Therefore, will try to search dependencies in MSG files first then in IDL files
    // via two separate recursive searches for each dependency.
    if (is_service_type) {
      format = Format::SRV;
      if (!topic_name.empty() && is_service_event_topic(topic_name, root_type)) {
        // Convert service event type to service type
        real_root_type = service_event_topic_type_to_service_type(root_type);
      }
    } else if (is_action_type) {
      format = Format::ACTION;
      if (!topic_name.empty() && is_topic_belong_to_action(topic_name, root_type)) {
        // Search for action type in cache first. Since we can't convert CancelGoalEvent or Status
        // action introspection interface types to the corresponding action type directly, we are
        // using cache to store the action type from other action introspection interface types
        // corresponding to the same topic name and original action type.
        std::string action_name = action_interface_name_to_action_name(topic_name);
        auto it = action_name_to_inner_action_interface_type_cache_.find(action_name);
        if (it != action_name_to_inner_action_interface_type_cache_.end() && !it->second.empty()) {
          real_root_type = it->second;
        } else {
          // Convert action interface type to action type
          std::string action_type = rosbag2_cpp::get_action_type_for_info(root_type);
          // Note: get_action_type_for_info(topic_type) will return empty string if the action
          // type is CancelGoalEvent or Status.
          if (!action_type.empty()) {
            real_root_type = std::move(action_type);
            action_name_to_inner_action_interface_type_cache_[action_name] = real_root_type;
          }
        }
      }
    }
    try {
      DefinitionIdentifier def_identifier{real_root_type, format};
      const MessageSpec & spec = load_message_spec(def_identifier);
      (void)seen_deps.insert(def_identifier).second;
      result = delimiter(def_identifier);
      result += spec.text;
      for (const auto & dep_name : spec.dependencies) {
        DefinitionIdentifier dep(dep_name, Format::MSG);
        bool inserted = seen_deps.insert(dep).second;
        if (inserted) {
          try {
            result += "\n";
            result += delimiter(dep);
            result += append_recursive(dep, max_recursion_depth);
            format = Format::MSG;
          } catch (const DefinitionNotFoundError & msg_search_err) {
            ROSBAG2_CPP_LOG_DEBUG("No .msg definition for %s, falling back to IDL",
                                  msg_search_err.what());
            try {
              dep = DefinitionIdentifier(dep_name, Format::IDL);
              inserted = seen_deps.insert(dep).second;
              if (inserted) {
                result += "\n";
                result += delimiter(dep);
                result += append_recursive(dep, max_recursion_depth);
                format = Format::IDL;
              }
            } catch (const DefinitionNotFoundError & idl_search_error) {
              ROSBAG2_CPP_LOG_DEBUG("No .idl definition found for topic type %s.",
                                    idl_search_error.what());
              format = Format::UNKNOWN;
            }
          } catch (const TypenameNotUnderstoodError & err) {
            ROSBAG2_CPP_LOG_DEBUG(
              "Message type name '%s' not understood by type definition search.", err.what());
            format = Format::UNKNOWN;
          }
        }
      }
    } catch (const DefinitionNotFoundError & real_root_search_err) {
      ROSBAG2_CPP_LOG_DEBUG("No message definition found for topic type %s.",
                            real_root_search_err.what());
      format = Format::UNKNOWN;
    } catch (const TypenameNotUnderstoodError & err) {
      ROSBAG2_CPP_LOG_DEBUG(
        "Message type name '%s' not understood by type definition search.", err.what());
      format = Format::UNKNOWN;
    }
  }
  rosbag2_storage::MessageDefinition out;
  switch (format) {
    case Format::UNKNOWN:
      out.encoding = "unknown";
      result = "";
      break;
    case Format::MSG:
    case Format::SRV:
    case Format::ACTION:
      out.encoding = "ros2msg";
      break;
    case Format::IDL:
      out.encoding = "ros2idl";
      break;
    default:
      throw std::runtime_error("switch is not exhaustive");
  }

  out.encoded_message_definition = result;
  out.topic_type = real_root_type;
  return out;
}
}  // namespace rosbag2_cpp
