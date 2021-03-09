// Copyright 2021 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#include "./rmw_implemented_serialization_format_converter.hpp"

#include <memory>
#include <sstream>
#include <string>

#include "ament_index_cpp/get_resources.hpp"
#include "rcpputils/find_library.hpp"
#include "rcpputils/shared_library.hpp"
#include "rmw/rmw.h"
#include "rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp"
#include "rosbag2_cpp/logging.hpp"

namespace
{
/// Get a function with a given signature from a shared library, with error checking.
template<typename T>
T get_function_from(const char * function_name, std::shared_ptr<rcpputils::SharedLibrary> library)
{
  if (!library->has_symbol(function_name)) {
    std::stringstream ss;
    ss << "Converter could not find expected symbol '" << function_name <<
      "' in rmw implementation library " << library->get_library_path();
    throw std::runtime_error{ss.str()};
  }
  // Function expected to return a value because has_symbol was checked first.
  T loaded_function = (T)(library->get_symbol(function_name));
  return loaded_function;
}
}  // namespace

namespace rosbag2_cpp
{

class RMWImplementedConverterImpl
{
public:
  explicit RMWImplementedConverterImpl(const std::string & format)
  {
    const std::string current_implementation_format{rmw_get_serialization_format()};
    if (current_implementation_format == format) {
      deserialize_fn_ = &rmw_deserialize;
      serialize_fn_ = &rmw_serialize;
      return;
    }

    auto packages_with_prefixes = ament_index_cpp::get_resources("rmw_typesupport");

    for (const auto & package_prefix_pair : packages_with_prefixes) {
      ROSBAG2_CPP_LOG_INFO_STREAM("Found package " << package_prefix_pair.first);
      const auto & pkg = package_prefix_pair.first;
      if (pkg == "rmw_implementation") {
        continue;
      }

      const auto libpath = rcpputils::find_library_path(pkg);
      if (libpath.empty()) {
        ROSBAG2_CPP_LOG_ERROR_STREAM(
          "Unexpectedly could not find library for RMW implementation " << pkg);
        continue;
      }
      library_ = std::make_shared<rcpputils::SharedLibrary>(libpath);

      auto get_format_fn = get_function_from<decltype(&rmw_get_serialization_format)>(
        "rmw_get_serialization_format", library_);
      const char * impl_serialization_format = get_format_fn();
      if (impl_serialization_format == format) {
        deserialize_fn_ = get_function_from<decltype(deserialize_fn_)>(
          "rmw_deserialize", library_);
        serialize_fn_ = get_function_from<decltype(serialize_fn_)>(
          "rmw_serialize", library_);
        return;
      }
    }

    throw std::runtime_error{
            std::string("No RMW implementation found supporting serialization format ") +
            format};
  }

  std::shared_ptr<rcpputils::SharedLibrary> library_;
  decltype(&rmw_serialize) serialize_fn_ = nullptr;
  decltype(&rmw_deserialize) deserialize_fn_ = nullptr;
};

RMWImplementedConverter::RMWImplementedConverter(const std::string & format)
: impl_(std::make_unique<RMWImplementedConverterImpl>(format))
{}

RMWImplementedConverter::~RMWImplementedConverter()
{}

void RMWImplementedConverter::deserialize(
  std::shared_ptr<const rosbag2_storage::SerializedBagMessage> serialized_message,
  const rosidl_message_type_support_t * type_support,
  std::shared_ptr<rosbag2_cpp::rosbag2_introspection_message_t> introspection_message)
{
  const auto ret = impl_->deserialize_fn_(
    serialized_message->serialized_data.get(), type_support, introspection_message->message);
  if (ret != RMW_RET_OK) {
    ROSBAG2_CPP_LOG_ERROR("Failed to deserialize message for conversion.");
  }
}

void RMWImplementedConverter::serialize(
  std::shared_ptr<const rosbag2_cpp::rosbag2_introspection_message_t> introspection_message,
  const rosidl_message_type_support_t * type_support,
  std::shared_ptr<rosbag2_storage::SerializedBagMessage> serialized_message)
{
  const auto ret = impl_->serialize_fn_(
    introspection_message->message, type_support, serialized_message->serialized_data.get());
  if (ret != RMW_RET_OK) {
    ROSBAG2_CPP_LOG_ERROR("Failed to re-serialize message for conversion.");
  }
}

}  // namespace rosbag2_cpp
