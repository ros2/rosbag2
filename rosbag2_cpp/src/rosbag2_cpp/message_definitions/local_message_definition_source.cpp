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

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iterator>
#include <optional>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_prefix.hpp>
#include "ament_index_cpp/get_resource.hpp"

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
/// name, optional middle path, and type name components.
/// \details The regex requires a format like package_name/[path/]TypeName where package_name and
/// TypeName consist of alphanumeric characters and underscores.
/// Breaking down this regex (^([a-zA-Z0-9_]+)/?(.*)/([a-zA-Z0-9_]+)$):
/// ^ - Ensures matching starts at beginning of string
/// ([a-zA-Z0-9_]+) - First capture group: matches the package name
/// (e.g., "std_msgs", "sensor_msgs")
/// /? - Optionally matches a forward slash after the package name
/// (.*) - Second capture group: matches any middle path segments (e.g., "msg", "srv/nested", etc.)
/// / - Matches a literal forward slash before the type name
/// ([a-zA-Z0-9_]+) - Third capture group: matches the type name (e.g., "String", "LaserScan")
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
///  String - missing package name separator
static const std::regex PACKAGE_TYPENAME_REGEX{
  R"(^([a-zA-Z0-9_]+)/?(.*)/([a-zA-Z0-9_]+)$)"
};
// Match field types from .msg, .srv and .action definitions
// ("foo_msgs/Bar" in "foo_msgs/Bar[] bar")
static const std::regex MSG_FIELD_TYPE_REGEX{R"((?:^|\n)\s*([a-zA-Z0-9_/]+)(?:\[[^\]]*\])?\s+)"};


/// \brief This regex is designed to parse IDL include statements to extract
/// message type dependencies.
/// \details Breaking down this regex:
/// (?:^|\n) - Ensures matching starts at the beginning of the string or after a newline
/// #include\s+ - Matches the literal string "#include" followed by one or more whitespace
/// characters
/// (?:\"|<) - Non-capturing group that matches either a double quote (") or a less-than sign (<)
/// ([a-zA-Z0-9_\/]+\/)? - Optional first capture group that matches the package path
/// (e.g., "foo_msgs/msg/") ending with a forward slash
/// ([a-zA-Z0-9_]+) - Second capture group that matches the type name (e.g., "Bar")
/// \.idl - Matches the literal string ".idl"
/// (?:\"|>) - Non-capturing group that matches either a double quote (") or a greater-than sign (>)
static const std::regex IDL_FIELD_TYPE_REGEX{
  R"((?:^|\n)#include\s+(?:\"|<)([a-zA-Z0-9_\/]+\/)?([a-zA-Z0-9_]+)\.idl(?:"|>))"};

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
    if (PRIMITIVE_TYPES.count(type)) {
      continue;
    }
    if (type.find('/') == std::string::npos) {
      dependencies.emplace(package_context + '/' + std::move(type));
    } else {
      dependencies.emplace(type);
    }
  }
  return dependencies;
}

/// \brief Parse IDL definition text to extract message type dependencies.
/// \details This function scans the provided IDL text for include statements
/// and extracts the message type dependencies. It supports both absolute and relative
/// type references. If a type is referenced without a package path, it is assumed to belong
/// to the specified package context.
/// \param text The IDL definition text to parse.
/// \param package_context The package context to use for relative type references.
/// \param namespace_hint An optional namespace hint to infix for relative type references
/// (e.g. "msg").
/// \return A set of message type dependencies in the format "package_name/msg/TypeName".
static std::set<std::string> parse_idl_dependencies(
  const std::string & text,
  const std::string & package_context = "",
  const std::string & namespace_hint = "")
{
  std::set<std::string> dependencies;

  for (std::sregex_iterator iter(text.begin(), text.end(), IDL_FIELD_TYPE_REGEX);
    iter != std::sregex_iterator(); ++iter)
  {
    const auto & pkg_path_capturing_group = (*iter)[1];
    const auto & type_name_capturing_group = (*iter)[2];

    if (pkg_path_capturing_group.matched && type_name_capturing_group.matched) {
      // If there is a path component, include it
      // Remove trailing "/" and append type name "foo_msgs/msg/" → "foo_msgs/msg/Bar"
      const auto & pkg_path_str = pkg_path_capturing_group.str();
      const auto & type_name_str = type_name_capturing_group.str();
      std::string dep = pkg_path_str.substr(0, pkg_path_str.length() - 1) + '/' + type_name_str;
      dependencies.emplace(dep);
    } else if (type_name_capturing_group.matched) {
      const auto & type_name_str = type_name_capturing_group.str();
      // No path: use current package context
      // "" → "current_package/Bar"
      std::string dep;
      if (!namespace_hint.empty()) {
        dep = package_context + '/' + namespace_hint + '/' + type_name_str;
      } else {
        dep = package_context + '/' + type_name_str;
      }
      dependencies.emplace(dep);
    }
  }
  return dependencies;
}

std::set<std::string> parse_definition_dependencies(
  LocalMessageDefinitionSource::Format format,
  const std::string & text,
  const std::string & package_context,
  const std::string & namespace_hint)
{
  switch (format) {
    case LocalMessageDefinitionSource::Format::MSG:
      return parse_msg_dependencies(text, package_context);
    case LocalMessageDefinitionSource::Format::IDL:
      return parse_idl_dependencies(text, package_context, namespace_hint);
    case LocalMessageDefinitionSource::Format::SRV:
    case LocalMessageDefinitionSource::Format::ACTION:
      {
        auto dep = parse_msg_dependencies(text, package_context);
        if (!dep.empty()) {
          return dep;
        } else {
          return parse_idl_dependencies(text, package_context, namespace_hint);
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

namespace
{
/// \brief Return the last namespace component of a type name, e.g. "srv" for "pkg/srv/Foo" or
/// "action" for "pkg/nested_sub_dir/action/Bar". Return an empty string if there is none.
std::string interface_namespace(const std::string & type)
{
  const auto name_pos = type.rfind('/');
  if (name_pos == std::string::npos || name_pos == 0) {
    return {};
  }
  const auto ns_pos = type.rfind('/', name_pos - 1);
  if (ns_pos == std::string::npos) {
    return {};
  }
  return type.substr(ns_pos + 1, name_pos - ns_pos - 1);
}

/// \brief A message type which rosidl generates from a .srv or .action file, and which therefore
/// has no definition file of its own.
struct GeneratedType
{
  /// The .srv or .action type it was generated from, e.g. "pkg/srv/Foo".
  std::string interface_type;
  /// The suffix appended to the interface type, e.g. "_Request".
  std::string suffix;
};

constexpr char SERVICE_EVENT_INFO_TYPE[] = "service_msgs/msg/ServiceEventInfo";
constexpr char REQUEST_SUFFIX[] = "_Request";
constexpr char RESPONSE_SUFFIX[] = "_Response";
constexpr char EVENT_SUFFIX[] = "_Event";
constexpr char GOAL_SUFFIX[] = "_Goal";
constexpr char RESULT_SUFFIX[] = "_Result";
constexpr char FEEDBACK_SUFFIX[] = "_Feedback";
constexpr char SEND_GOAL_SUFFIX[] = "_SendGoal";
constexpr char GET_RESULT_SUFFIX[] = "_GetResult";
constexpr char FEEDBACK_MESSAGE_SUFFIX[] = "_FeedbackMessage";

/// \brief Split a type generated from a .srv or .action file into the type it was generated from
/// and the suffix rosidl appended to it.
/// \details Besides the real message types, the action-internal service types `<Action>_SendGoal`
/// and `<Action>_GetResult` are recognized as well. They are used to name the synthesized IDL
/// file holding their request and response structures, the same way as `pkg/srv/Foo.idl` holds
/// the `Foo_Request` and `Foo_Response` structures.
std::optional<GeneratedType> parse_generated_type(const std::string & type)
{
  static const std::vector<std::string> service_suffixes{
    REQUEST_SUFFIX, RESPONSE_SUFFIX, EVENT_SUFFIX};
  static const std::vector<std::string> action_suffixes{
    std::string(SEND_GOAL_SUFFIX) + REQUEST_SUFFIX,
    std::string(SEND_GOAL_SUFFIX) + RESPONSE_SUFFIX,
    std::string(SEND_GOAL_SUFFIX) + EVENT_SUFFIX,
    std::string(GET_RESULT_SUFFIX) + REQUEST_SUFFIX,
    std::string(GET_RESULT_SUFFIX) + RESPONSE_SUFFIX,
    std::string(GET_RESULT_SUFFIX) + EVENT_SUFFIX,
    FEEDBACK_MESSAGE_SUFFIX, SEND_GOAL_SUFFIX, GET_RESULT_SUFFIX,
    GOAL_SUFFIX, RESULT_SUFFIX, FEEDBACK_SUFFIX};

  const std::string ns = interface_namespace(type);
  const std::vector<std::string> * suffixes = nullptr;
  if (ns == "srv") {
    suffixes = &service_suffixes;
  } else if (ns == "action") {
    suffixes = &action_suffixes;
  } else {
    return std::nullopt;
  }
  const std::string name = type.substr(type.rfind('/') + 1);
  for (const auto & suffix : *suffixes) {
    if (name.size() > suffix.size() &&
      name.compare(name.size() - suffix.size(), suffix.size(), suffix) == 0)
    {
      return GeneratedType{type.substr(0, type.size() - suffix.size()), suffix};
    }
  }
  return std::nullopt;
}

/// \brief Split the text of a .srv or .action file into its sections, which are separated by
/// lines consisting of "---".
/// \return The sections. Each non-empty section ends with a new line.
std::vector<std::string> split_interface_sections(const std::string & text)
{
  std::vector<std::string> sections(1);
  std::istringstream stream(text);
  std::string line;
  while (std::getline(stream, line)) {
    const auto end = line.find_last_not_of(" \t\r");
    if (end != std::string::npos && line.compare(0, end + 1, "---") == 0) {
      sections.emplace_back();
    } else {
      sections.back() += line + "\n";
    }
  }
  return sections;
}

std::string section_or_empty(const std::vector<std::string> & sections, size_t index)
{
  return index < sections.size() ? sections[index] : std::string{};
}

/// \brief Text of a service event message in .msg format, as generated by rosidl for the service.
std::string service_event_msg_text(const std::string & service_type)
{
  return std::string(SERVICE_EVENT_INFO_TYPE) + " info\n" +
         service_type + REQUEST_SUFFIX + "[<=1] request\n" +
         service_type + RESPONSE_SUFFIX + "[<=1] response\n";
}

/// \brief Convert a type name to an IDL scoped name, e.g. "pkg/msg/Foo" to "pkg::msg::Foo".
std::string idl_scoped_name(const std::string & type)
{
  return std::regex_replace(type, std::regex("/"), "::");
}

struct IdlStruct
{
  std::string name;
  /// Pairs of IDL member type and member name.
  std::vector<std::pair<std::string, std::string>> members;
};

/// \brief Render IDL text defining the given structures in the modules of namespace_path
/// (e.g. "pkg/srv"), in the same layout as generated by rosidl_adapter.
std::string idl_text(
  const std::set<std::string> & included_types,
  const std::string & namespace_path,
  const std::vector<IdlStruct> & structs)
{
  std::string text;
  for (const auto & included_type : included_types) {
    text += "#include \"" + included_type + ".idl\"\n";
  }
  text += "\n";
  std::vector<std::string> modules;
  std::istringstream stream(namespace_path);
  for (std::string module; std::getline(stream, module, '/'); ) {
    modules.push_back(module);
  }
  std::string indent;
  for (const auto & module : modules) {
    text += indent + "module " + module + " {\n";
    indent += "  ";
  }
  for (const auto & idl_struct : structs) {
    text += indent + "struct " + idl_struct.name + " {\n";
    for (const auto & [member_type, member_name] : idl_struct.members) {
      text += indent + "  " + member_type + " " + member_name + ";\n";
    }
    text += indent + "};\n";
  }
  for (size_t i = 0; i < modules.size(); ++i) {
    indent.resize(indent.size() - 2);
    text += indent + "};\n";
  }
  return text;
}

/// \brief Text of a service event message in .idl format, as generated by rosidl for the service.
/// \details The request and response structures are expected to be defined in the IDL file named
/// after the service type, e.g. "pkg/srv/Foo.idl".
std::string service_event_idl_text(const std::string & service_type)
{
  const std::string namespace_path = service_type.substr(0, service_type.rfind('/'));
  const std::string name = service_type.substr(service_type.rfind('/') + 1);
  return idl_text(
    {SERVICE_EVENT_INFO_TYPE, service_type}, namespace_path,
    {{name + EVENT_SUFFIX, {
        {idl_scoped_name(SERVICE_EVENT_INFO_TYPE), "info"},
        {"sequence<" + idl_scoped_name(service_type + REQUEST_SUFFIX) + ", 1>", "request"},
        {"sequence<" + idl_scoped_name(service_type + RESPONSE_SUFFIX) + ", 1>", "response"}}}});
}
}  // namespace

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
  const std::string & package_context, const std::string & namespace_hint)
: dependencies(parse_definition_dependencies(format, text, package_context, namespace_hint)),
  text(std::move(text)),
  format(format)
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
  if (const MessageSpec * synthesized_spec = synthesize_message_spec(definition_identifier)) {
    return *synthesized_spec;
  }
  std::smatch match;
  const std::string topic_type = definition_identifier.topic_type();
  if (!std::regex_match(topic_type, match, PACKAGE_TYPENAME_REGEX) || match.size() < 3) {
    throw TypenameNotUnderstoodError(topic_type);
  }
  const std::string package_name{match[1].str()};
  const std::string subdir{match[2].str()};
  const std::string type_name{match[3].str()};
  const std::string type_name_with_extension =
    type_name + extension_for_format(definition_identifier.format());
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
      if (curr_relative_file_path.filename() == type_name_with_extension) {
        relative_file_path_str = curr_relative_file_path.generic_string();
        break;
      }
    }
  }

  if (relative_file_path_str.empty()) {
    ROSBAG2_CPP_LOG_DEBUG(
      "Message definition file '%s' not found in the resource content for package: '%s'",
      type_name_with_extension.c_str(), package_name.c_str());
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
    MessageSpec(definition_identifier.format(), std::move(contents), package_name,
      subdir)).first->second;

  // "References and pointers to data stored in the container are only invalidated by erasing that
  // element, even when the corresponding iterator is invalidated."
  return spec;
}

const LocalMessageDefinitionSource::MessageSpec *
LocalMessageDefinitionSource::synthesize_message_spec(
  const DefinitionIdentifier & definition_identifier)
{
  const std::string type = definition_identifier.topic_type();
  const auto generated_type = parse_generated_type(type);
  if (!generated_type) {
    return nullptr;
  }
  const std::string & interface_type = generated_type->interface_type;
  const std::string & suffix = generated_type->suffix;
  const bool is_action = interface_namespace(type) == "action";
  const std::string uuid_type = "unique_identifier_msgs/msg/UUID";
  const std::string time_type = "builtin_interfaces/msg/Time";

  std::optional<std::string> text;
  if (definition_identifier.format() == Format::MSG) {
    // Field types of the synthesized definitions are written the same way as in the .msg files
    // of action_msgs (e.g. "unique_identifier_msgs/UUID") to share their dependency sections.
    auto section = [&](Format interface_format, size_t index) {
        return section_or_empty(
          split_interface_sections(
            load_message_spec(DefinitionIdentifier(interface_type, interface_format)).text),
          index);
      };
    if (!is_action) {
      if (suffix == REQUEST_SUFFIX) {
        text = section(Format::SRV, 0);
      } else if (suffix == RESPONSE_SUFFIX) {
        text = section(Format::SRV, 1);
      } else if (suffix == EVENT_SUFFIX) {
        text = service_event_msg_text(interface_type);
      }
    } else if (suffix == GOAL_SUFFIX) {
      text = section(Format::ACTION, 0);
    } else if (suffix == RESULT_SUFFIX) {
      text = section(Format::ACTION, 1);
    } else if (suffix == FEEDBACK_SUFFIX) {
      text = section(Format::ACTION, 2);
    } else if (suffix == std::string(SEND_GOAL_SUFFIX) + REQUEST_SUFFIX) {
      text = "unique_identifier_msgs/UUID goal_id\n" + interface_type + GOAL_SUFFIX + " goal\n";
    } else if (suffix == std::string(SEND_GOAL_SUFFIX) + RESPONSE_SUFFIX) {
      text = "bool accepted\nbuiltin_interfaces/Time stamp\n";
    } else if (suffix == std::string(SEND_GOAL_SUFFIX) + EVENT_SUFFIX) {
      text = service_event_msg_text(interface_type + SEND_GOAL_SUFFIX);
    } else if (suffix == std::string(GET_RESULT_SUFFIX) + REQUEST_SUFFIX) {
      text = "unique_identifier_msgs/UUID goal_id\n";
    } else if (suffix == std::string(GET_RESULT_SUFFIX) + RESPONSE_SUFFIX) {
      text = "int8 status\n" + interface_type + RESULT_SUFFIX + " result\n";
    } else if (suffix == std::string(GET_RESULT_SUFFIX) + EVENT_SUFFIX) {
      text = service_event_msg_text(interface_type + GET_RESULT_SUFFIX);
    } else if (suffix == FEEDBACK_MESSAGE_SUFFIX) {
      text = "unique_identifier_msgs/UUID goal_id\n" + interface_type + FEEDBACK_SUFFIX +
        " feedback\n";
    }
  } else if (definition_identifier.format() == Format::IDL) {
    // Request, response, goal, result and feedback structures of .srv and .action files are
    // defined in the IDL files generated from them by rosidl_adapter, e.g. "pkg/srv/Foo.idl"
    // and "pkg/action/Bar.idl". Only the structures which rosidl generates on the fly need to be
    // synthesized.
    const std::string namespace_path = type.substr(0, type.rfind('/'));
    const std::string interface_name = interface_type.substr(interface_type.rfind('/') + 1);
    if (!is_action) {
      if (suffix == EVENT_SUFFIX) {
        text = service_event_idl_text(interface_type);
      }
    } else if (suffix == SEND_GOAL_SUFFIX) {
      const std::string service_name = interface_name + SEND_GOAL_SUFFIX;
      text = idl_text(
        {interface_type, time_type, uuid_type}, namespace_path,
        {{service_name + REQUEST_SUFFIX, {
            {idl_scoped_name(uuid_type), "goal_id"},
            {idl_scoped_name(interface_type + GOAL_SUFFIX), "goal"}}},
          {service_name + RESPONSE_SUFFIX, {
              {"boolean", "accepted"},
              {idl_scoped_name(time_type), "stamp"}}}});
    } else if (suffix == GET_RESULT_SUFFIX) {
      const std::string service_name = interface_name + GET_RESULT_SUFFIX;
      text = idl_text(
        {interface_type, uuid_type}, namespace_path,
        {{service_name + REQUEST_SUFFIX, {
            {idl_scoped_name(uuid_type), "goal_id"}}},
          {service_name + RESPONSE_SUFFIX, {
              {"int8", "status"},
              {idl_scoped_name(interface_type + RESULT_SUFFIX), "result"}}}});
    } else if (suffix == std::string(SEND_GOAL_SUFFIX) + EVENT_SUFFIX) {
      text = service_event_idl_text(interface_type + SEND_GOAL_SUFFIX);
    } else if (suffix == std::string(GET_RESULT_SUFFIX) + EVENT_SUFFIX) {
      text = service_event_idl_text(interface_type + GET_RESULT_SUFFIX);
    } else if (suffix == FEEDBACK_MESSAGE_SUFFIX) {
      text = idl_text(
        {interface_type, uuid_type}, namespace_path,
        {{interface_name + FEEDBACK_MESSAGE_SUFFIX, {
            {idl_scoped_name(uuid_type), "goal_id"},
            {idl_scoped_name(interface_type + FEEDBACK_SUFFIX), "feedback"}}}});
    }
  }
  if (!text) {
    return nullptr;
  }

  std::smatch match;
  std::string package_name;
  if (std::regex_match(type, match, PACKAGE_TYPENAME_REGEX)) {
    package_name = match[1].str();
  }
  return &msg_specs_by_definition_identifier_.emplace(
    definition_identifier,
    MessageSpec(definition_identifier.format(), std::move(*text), package_name, "")
  ).first->second;
}

rosbag2_storage::MessageDefinition LocalMessageDefinitionSource::get_full_text(
  const std::string & root_type)
{
  return get_full_text_ext(root_type, std::string{});
}

rosbag2_storage::MessageDefinition LocalMessageDefinitionSource::get_full_text_ext(
  const std::string & root_type,
  const std::string & /*topic_name*/)
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

  const int32_t max_recursion_depth =
    ROSBAG2_CPP_LOCAL_MESSAGE_DEFINITION_SOURCE_MAX_RECURSION_DEPTH;

  // Resolve root_type as a message type: as a complete .msg definition first, otherwise as a
  // complete .idl definition. Message types generated from .srv and .action files (e.g. service
  // event types used by the service introspection topics) are synthesized by load_message_spec().
  auto get_message_full_text = [&](std::string & result) -> Format {
      try {
        seen_deps.clear();
        // Note: By design The top-level message definition for MSG format is present first, with
        // no delimiter. All dependent .msg definitions are preceded by a two-line delimiter.
        result = append_recursive(DefinitionIdentifier(root_type, Format::MSG),
                                  max_recursion_depth);
        return Format::MSG;
      } catch (const DefinitionNotFoundError & err) {
        ROSBAG2_CPP_LOG_DEBUG("No .msg definition for %s, falling back to IDL", err.what());
      } catch (const TypenameNotUnderstoodError & err) {
        ROSBAG2_CPP_LOG_DEBUG(
          "Message type name '%s' not understood by type definition search.", err.what());
        return Format::UNKNOWN;
      }
      try {
        seen_deps.clear();
        DefinitionIdentifier root_definition_identifier(root_type, Format::IDL);
        (void)seen_deps.insert(root_definition_identifier);
        result = (delimiter(root_definition_identifier) +
          append_recursive(root_definition_identifier, max_recursion_depth));
        return Format::IDL;
      } catch (const DefinitionNotFoundError & idl_search_error) {
        ROSBAG2_CPP_LOG_DEBUG("No .idl definition found for topic type %s.",
                              idl_search_error.what());
      } catch (const TypenameNotUnderstoodError & err) {
        ROSBAG2_CPP_LOG_DEBUG(
          "Message type name '%s' not understood by type definition search.", err.what());
      }
      return Format::UNKNOWN;
    };

  // Resolve root_type as a plain service or action type, i.e. the content of the .srv or .action
  // file followed by its dependencies. The dependencies could be either in the msg or idl files.
  // Therefore, will try to search dependencies in MSG files first then in IDL files.
  auto get_interface_full_text = [&](Format interface_format, std::string & result) -> Format {
      seen_deps.clear();
      bool has_idl_dependency = false;
      try {
        DefinitionIdentifier def_identifier{root_type, interface_format};
        const MessageSpec & spec = load_message_spec(def_identifier);
        (void)seen_deps.insert(def_identifier);
        result = delimiter(def_identifier) + spec.text;
        for (const auto & dep_name : spec.dependencies) {
          DefinitionIdentifier msg_dep(dep_name, Format::MSG);
          if (!seen_deps.insert(msg_dep).second) {
            continue;
          }
          try {
            std::string dep_text = append_recursive(msg_dep, max_recursion_depth);
            result += "\n" + delimiter(msg_dep) + dep_text;
            continue;
          } catch (const DefinitionNotFoundError & msg_search_err) {
            ROSBAG2_CPP_LOG_DEBUG("No .msg definition for %s, falling back to IDL",
                                  msg_search_err.what());
          }
          DefinitionIdentifier idl_dep(dep_name, Format::IDL);
          if (seen_deps.insert(idl_dep).second) {
            std::string dep_text = append_recursive(idl_dep, max_recursion_depth);
            result += "\n" + delimiter(idl_dep) + dep_text;
            has_idl_dependency = true;
          }
        }
      } catch (const DefinitionNotFoundError & search_err) {
        ROSBAG2_CPP_LOG_DEBUG("No definition found for %s.", search_err.what());
        return Format::UNKNOWN;
      } catch (const TypenameNotUnderstoodError & err) {
        ROSBAG2_CPP_LOG_DEBUG(
          "Message type name '%s' not understood by type definition search.", err.what());
        return Format::UNKNOWN;
      }
      return has_idl_dependency ? Format::IDL : interface_format;
    };

  std::string result;
  Format format = Format::UNKNOWN;
  const std::string ns = interface_namespace(root_type);
  const bool is_interface_type = (ns == "srv" || ns == "action") &&
    !parse_generated_type(root_type).has_value();

  if (!is_interface_type) {
    format = get_message_full_text(result);
  }
  if (format == Format::UNKNOWN && (ns == "srv" || ns == "action")) {
    // Note: A service or action named like a generated type (e.g. "pkg/srv/Foo_Event") is
    // resolved here if no such generated type could be resolved above.
    format = get_interface_full_text(ns == "srv" ? Format::SRV : Format::ACTION, result);
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
  out.topic_type = root_type;
  return out;
}
}  // namespace rosbag2_cpp
