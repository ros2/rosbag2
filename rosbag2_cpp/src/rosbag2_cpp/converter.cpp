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

#include "rosbag2_cpp/converter.hpp"

#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_resource.hpp"
#include "ament_index_cpp/get_resources.hpp"
#include "rcpputils/find_library.hpp"
#include "rmw/rmw.h"
#include "rosbag2_cpp/info.hpp"
#include "rosbag2_cpp/typesupport_helpers.hpp"
#include "rosbag2_cpp/logging.hpp"
#include "rosbag2_storage/metadata_io.hpp"
#include "rosbag2_storage/ros_helper.hpp"
#include "rosbag2_storage/storage_options.hpp"

namespace
{
// Convenience struct to keep both type supports (rmw and introspection) together.
struct ConverterTypeSupport
{
  std::shared_ptr<rcpputils::SharedLibrary> type_support_library;
  const rosidl_message_type_support_t * rmw_type_support;

  std::shared_ptr<rcpputils::SharedLibrary> introspection_type_support_library;
  const rosidl_message_type_support_t * introspection_type_support;
};

template<typename T>
T get_function_from(const char * function_name, std::shared_ptr<rcpputils::SharedLibrary> library)
{
  if (!library->has_symbol(function_name)) {
    std::stringstream ss;
    ss << "Converter could not find expected symbol '" << function_name <<
      "' in rmw implementation " << library->get_library_path();
    throw std::runtime_error{ss.str()};
  }
  T loaded_function = nullptr;
  // Function expected to return a value because has_symbol was checked first.
  loaded_function = (decltype(loaded_function))(library->get_symbol(function_name));
  return loaded_function;
}
}  // namespace


namespace rosbag2_cpp
{

typedef std::shared_ptr<rosbag2_storage::SerializedBagMessage> Msg;
typedef std::shared_ptr<const rosbag2_storage::SerializedBagMessage> ConstMsg;

class ConverterImpl
{
public:
  explicit ConverterImpl(const ConverterOptions & options)
  {
    const std::string current_implementation_format{rmw_get_serialization_format()};
    if (current_implementation_format == options.input_serialization_format) {
      deserialize_fn_ = &rmw_deserialize;
      ROSBAG2_CPP_LOG_INFO("The current impl is the DESER");
    }
    if (current_implementation_format == options.output_serialization_format) {
      serialize_fn_ = &rmw_serialize;
      ROSBAG2_CPP_LOG_INFO("The current impl is the SER");
    }

    auto packages_with_prefixes = ament_index_cpp::get_resources("rmw_typesupport");

    for (const auto & package_prefix_pair : packages_with_prefixes) {
      ROSBAG2_CPP_LOG_ERROR_STREAM("YOW A PAKAJ " << package_prefix_pair.first);
      if (serialize_fn_ && deserialize_fn_) {
        break;
      }

      const auto & pkg = package_prefix_pair.first;
      if (pkg == "rmw_implementation") {
        continue;
      }

      const auto libpath = rcpputils::find_library_path(pkg);
      if (libpath.empty()) {
        ROSBAG2_CPP_LOG_ERROR_STREAM("COULD NOT FIND LIB FOR " << pkg);
        continue;
      }
      auto library = std::make_shared<rcpputils::SharedLibrary>(libpath);

      auto get_format_fn = get_function_from<decltype(&rmw_get_serialization_format)>(
        "rmw_get_serialization_format", library);
      const char * fmt = get_format_fn();
      if (!serialize_fn_ && fmt == options.input_serialization_format) {
        deserialize_fn_ = get_function_from<decltype(deserialize_fn_)>(
          "rmw_deserialize", library);
      } else if (!deserialize_fn_ && fmt == options.output_serialization_format) {
        serialize_fn_ = get_function_from<decltype(serialize_fn_)>(
          "rmw_serialize", library);
      }
    }

    if (!deserialize_fn_) {
      throw std::runtime_error{
              std::string("No implementation could be found for deserializing from format ") +
              options.input_serialization_format};
    }
    if (!serialize_fn_) {
      throw std::runtime_error{
              std::string("No implementation could be found for serializing to format ") +
              options.output_serialization_format};
    }
  }

  Msg convert(ConstMsg message)
  {
    auto ts = topics_and_types_.at(message->topic_name).rmw_type_support;
    auto introspection_ts = topics_and_types_.at(message->topic_name).introspection_type_support;
    auto allocator = rcutils_get_default_allocator();
    std::shared_ptr<rosbag2_introspection_message_t> introspection_message =
      allocate_introspection_message(introspection_ts, &allocator);
    auto output_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();

    // deserialize
    rosbag2_cpp::introspection_message_set_topic_name(
      introspection_message.get(), message->topic_name.c_str());
    introspection_message->time_stamp = message->time_stamp;

    auto ret = deserialize_fn_(
      message->serialized_data.get(), ts, introspection_message->message);
    if (ret != RMW_RET_OK) {
      ROSBAG2_CPP_LOG_ERROR("Failed to deserialize message for conversion.");
      return nullptr;
    }

    // re-serialize
    output_message->serialized_data = rosbag2_storage::make_empty_serialized_message(0);
    output_message->topic_name = std::string(introspection_message->topic_name);
    output_message->time_stamp = introspection_message->time_stamp;

    ret = serialize_fn_(
      introspection_message->message, ts, output_message->serialized_data.get());
    if (ret != RMW_RET_OK) {
      ROSBAG2_CPP_LOG_ERROR("Failed to re-serialize message for conversion.");
      return nullptr;
    }
    return output_message;
  }

  // One of serialize/deserialize is provided by the currently loaded RMW implementation.
  // The other will come from a loaded library - this variable stores that
  std::shared_ptr<rcpputils::SharedLibrary> serialization_library_;
  decltype(&rmw_serialize) serialize_fn_ = nullptr;
  decltype(&rmw_deserialize) deserialize_fn_ = nullptr;

  std::unordered_map<std::string, ConverterTypeSupport> topics_and_types_;
};

Converter::Converter(
  const std::string & input_format,
  const std::string & output_format)
: Converter(ConverterOptions{input_format, output_format})
{}

Converter::Converter(const ConverterOptions & converter_options)
: impl_(std::make_unique<ConverterImpl>(converter_options))
{
}

Converter::~Converter()
{
}

Msg Converter::convert(ConstMsg message) const
{
  return impl_->convert(message);
}

void Converter::add_topic(const std::string & topic, const std::string & type)
{
  ConverterTypeSupport type_support;

  type_support.type_support_library = get_typesupport_library(
    type, "rosidl_typesupport_cpp");
  type_support.rmw_type_support = get_typesupport_handle(
    type, "rosidl_typesupport_cpp",
    type_support.type_support_library);

  type_support.introspection_type_support_library = get_typesupport_library(
    type, "rosidl_typesupport_introspection_cpp");
  type_support.introspection_type_support = get_typesupport_handle(
    type, "rosidl_typesupport_introspection_cpp",
    type_support.introspection_type_support_library);

  impl_->topics_and_types_.insert({topic, type_support});
}

}  // namespace rosbag2_cpp
