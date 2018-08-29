// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#ifndef UTILS__TYPESUPPORT_UTILS_HPP_
#define UTILS__TYPESUPPORT_UTILS_HPP_

#include <memory>
#include <string>

#include "ament_index_cpp/get_package_prefix.hpp"

#include "Poco/SharedLibrary.h"

#include "rosidl_generator_c/message_type_support_struct.h"

namespace utils
{

std::string get_typesupport_library(
  const std::string package_name, const std::string & typesupport_identifier)
{
  const char * filename_prefix;
  const char * filename_extension;
#ifdef _WIN32
  filename_prefix = "";
  filename_extension = ".dll";
#elif __APPLE__
  filename_prefix = "lib";
  filename_extension = ".dylib";
#else
  filename_prefix = "lib";
  filename_extension = ".so";
#endif

  auto package_prefix = ament_index_cpp::get_package_prefix(package_name);
  auto library_path = package_prefix + "/lib/" +
    filename_prefix + package_name + "__" + typesupport_identifier + filename_extension;
  fprintf(stderr, "package prefix for %s: %s\n", package_name.c_str(), package_prefix.c_str());
  fprintf(stderr, "full path to library: %s\n", library_path.c_str());

  return library_path;
}

std::string get_typesupport_c_library(const std::string & package_name)
{
  return get_typesupport_library(package_name, "__rosidl_typesupport_c");
}

std::string get_typesupport_cpp_library(const std::string & package_name)
{
  return get_typesupport_library(package_name, "__rosidl_typesupport_cpp");
}

const rosidl_message_type_support_t * get_typesupport(
  const std::string & package_name,
  const std::string & message_name,
  const std::string & typesupport_identifier)
{
  auto library_path = get_typesupport_library(package_name, typesupport_identifier);
  std::shared_ptr<Poco::SharedLibrary> typesupport_library = nullptr;
  const rosidl_message_type_support_t * ts = nullptr;

  try {
    typesupport_library = std::make_shared<Poco::SharedLibrary>(library_path);
    auto symbol_name = typesupport_identifier + "__get_message_type_support_handle__" +
      package_name + "__msg__" + message_name;
    if (!typesupport_library->hasSymbol(symbol_name)) {
      fprintf(stderr, "symbol not found %s\n", symbol_name.c_str());
      return ts;
    }

    const rosidl_message_type_support_t * (* get_ts_func)(void) = nullptr;
    auto get_ts = (decltype(get_ts_func))typesupport_library->getSymbol(symbol_name);
    ts = get_ts();
    if (!ts) {
      fprintf(stderr, "ts is null\n");
      return ts;
    }
  } catch (Poco::LibraryLoadException & e) {
    fprintf(stderr, "poco exception: %s\n", e.what());
    return ts;
  }

  return ts;
}

const rosidl_message_type_support_t * get_typesupport_c(
  const std::string & package_name, const std::string & message_name)
{
  return get_typesupport(package_name, message_name, "rosidl_typesupport_c");
}

const rosidl_message_type_support_t * get_typesupport_cpp(
  const std::string & package_name, const std::string & message_name)
{
  return get_typesupport(package_name, message_name, "rosidl_typesupport_cpp");
}

}  // namespace utils

#endif  // UTILS__TYPESUPPORT_UTILS_HPP_
