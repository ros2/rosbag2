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

#include "cdr_converter.hpp"

#include <memory>
#include <string>

#include "ament_index_cpp/get_resources.hpp"
#include "ament_index_cpp/get_package_prefix.hpp"

#include "Poco/SharedLibrary.h"

#include "rcutils/strdup.h"
#include "rosbag2/types/introspection_message.hpp"
#include "rosidl_generator_cpp/message_type_support_decl.hpp"

#include "../logging.hpp"

namespace rosbag2_converter_default_plugins
{

inline
std::string get_package_library_path(const std::string & package_name)
{
  const char * filename_prefix;
  const char * filename_extension;
  const char * dynamic_library_folder;
#ifdef _WIN32
  filename_prefix = "";
  filename_extension = ".dll";
  dynamic_library_folder = "/bin/";
#elif __APPLE__
  filename_prefix = "lib";
  filename_extension = ".dylib";
  dynamic_library_folder = "/lib/";
#else
  filename_prefix = "lib";
  filename_extension = ".so";
  dynamic_library_folder = "/lib/";
#endif

  auto package_prefix = ament_index_cpp::get_package_prefix(package_name);
  auto library_path = package_prefix + dynamic_library_folder + filename_prefix +
    package_name + filename_extension;

  return library_path;
}

CdrConverter::CdrConverter()
{
  auto library_path = get_package_library_path("rmw_fastrtps_dynamic_cpp");
  std::shared_ptr<Poco::SharedLibrary> library;
  try {
    library = std::make_shared<Poco::SharedLibrary>(library_path);
  } catch (Poco::LibraryLoadException &) {
    throw std::runtime_error(
            std::string("poco exception: library could not be found:") + library_path);
  }

  std::string serialize_symbol = "rmw_serialize";
  std::string deserialize_symbol = "rmw_deserialize";

  if (!library->hasSymbol(serialize_symbol)) {
    throw std::runtime_error(
            std::string("poco exception: symbol not found: ") + serialize_symbol);
  }

  if (!library->hasSymbol(deserialize_symbol)) {
    throw std::runtime_error(
            std::string("poco exception: symbol not found: ") + deserialize_symbol);
  }

  serialize_fcn_ = (decltype(serialize_fcn_))library->getSymbol(serialize_symbol);
  if (!serialize_fcn_) {
    throw std::runtime_error(
            std::string("poco exception: symbol of wrong type: ") + serialize_symbol);
  }

  deserialize_fcn_ = (decltype(deserialize_fcn_))library->getSymbol(deserialize_symbol);
  if (!deserialize_fcn_) {
    throw std::runtime_error(
            std::string("poco exception: symbol of wrong type: ") + deserialize_symbol);
  }
}

void CdrConverter::deserialize(
  const std::shared_ptr<const rosbag2::SerializedBagMessage> serialized_message,
  const rosidl_message_type_support_t * type_support,
  std::shared_ptr<rosbag2_introspection_message_t> introspection_message)
{
  rosbag2::introspection_message_set_topic_name(
    introspection_message.get(), serialized_message->topic_name.c_str());
  introspection_message->time_stamp = serialized_message->time_stamp;

  auto ret = deserialize_fcn_(
    serialized_message->serialized_data.get(), type_support, introspection_message->message);
  if (ret != RMW_RET_OK) {
    ROSBAG2_CONVERTER_DEFAULT_PLUGINS_LOG_ERROR("Failed to deserialize message.");
  }
}

void CdrConverter::serialize(
  const std::shared_ptr<const rosbag2_introspection_message_t> introspection_message,
  const rosidl_message_type_support_t * type_support,
  std::shared_ptr<rosbag2::SerializedBagMessage> serialized_message)
{
  serialized_message->topic_name = std::string(introspection_message->topic_name);
  serialized_message->time_stamp = introspection_message->time_stamp;

  auto ret = serialize_fcn_(
    introspection_message->message, type_support, serialized_message->serialized_data.get());
  if (ret != RMW_RET_OK) {
    ROSBAG2_CONVERTER_DEFAULT_PLUGINS_LOG_ERROR("Failed to serialize message.");
  }
}

}  // namespace rosbag2_converter_default_plugins

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(rosbag2_converter_default_plugins::CdrConverter,
  rosbag2::converter_interfaces::SerializationFormatConverter)
