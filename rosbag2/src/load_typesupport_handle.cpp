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

#include <unistd.h>

#include "ament_index_cpp/get_resources.hpp"
#include "ament_index_cpp/get_package_prefix.hpp"

#include "Poco/SharedLibrary.h"

#include "rcl/rcl.h"

#include "rcutils/snprintf.h"

#include "rosidl_generator_c/message_type_support_struct.h"

std::string get_typesupport_library(const std::string & package_name)
{
  const char * env_var;
  char separator;
  const char * filename_prefix;
  const char * filename_extension;
#ifdef _WIN32
  env_var = "PATH";
  separator = ';';
  filename_prefix = "";
  filename_extension = ".dll";
#elif __APPLE__
  env_var = "DYLD_LIBRARY_PATH";
  separator = ':';
  filename_prefix = "lib";
  filename_extension = ".dylib";
#else
  env_var = "LD_LIBRARY_PATH";
  separator = ':';
  filename_prefix = "lib";
  filename_extension = ".so";
#endif

  auto package_prefix = ament_index_cpp::get_package_prefix(package_name);
  auto library_path = package_prefix + "/lib/" + filename_prefix + package_name + "__rosidl_typesupport_c" + filename_extension;
  fprintf(stderr, "package prefix for %s: %s\n", package_name.c_str(), package_prefix.c_str());
  fprintf(stderr, "full path to library: %s\n", library_path.c_str());

  return library_path;
}

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  std::string type_name = "String";
  std::string package_name = "std_msgs";

  auto library_path = get_typesupport_library(package_name);
  std::shared_ptr<Poco::SharedLibrary> typesupport_library = nullptr;
  const rosidl_message_type_support_t * ts = nullptr;

  try {
    typesupport_library = std::make_shared<Poco::SharedLibrary>(library_path);
    auto symbol_name = std::string("rosidl_typesupport_c__get_message_type_support_handle__")  + package_name + "__msg__" + type_name;
    fprintf(stderr, "sumbol name %s\n", symbol_name.c_str());
    if (!typesupport_library->hasSymbol(symbol_name)) {
      fprintf(stderr, "symbol not found %s\n", symbol_name.c_str());
      return -1;
    }
    fprintf(stderr, "symbol found\n");

    const rosidl_message_type_support_t * (*get_ts)(void) = nullptr;
    get_ts = (decltype(get_ts)) typesupport_library->getSymbol(symbol_name);
    ts = get_ts();
    if (!ts) {
      fprintf(stderr, "ts is null\n");
      return -1;
    }
    fprintf(stderr, "ts identifier %s\n", ts->typesupport_identifier);
  } catch (Poco::LibraryLoadException & e) {
    fprintf(stderr, "poco exception: %s\n", e.what());
  }

  auto ret = RCL_RET_ERROR;

  ret = rcl_init(0, nullptr, rcl_get_default_allocator());
  auto node_ptr = new rcl_node_t;
  *node_ptr = rcl_get_zero_initialized_node();
  const char * name = "node_name";
  rcl_node_options_t node_options = rcl_node_get_default_options();
  ret = rcl_node_init(node_ptr, name, "", &node_options);

  rcl_publisher_t publisher = rcl_get_zero_initialized_publisher();
  rcl_publisher_options_t publisher_options = rcl_publisher_get_default_options();
  ret = rcl_publisher_init(&publisher, node_ptr, ts, "topic_name", &publisher_options);

  rmw_serialized_message_t serialized_msg = rmw_get_zero_initialized_serialized_message();
  auto allocator = rcutils_get_default_allocator();
  auto initial_capacity = strlen("Hello World") + 10u;
  ret = rmw_serialized_message_init(
    &serialized_msg,
    initial_capacity,
    &allocator);
  if (ret != RCL_RET_OK) {
    throw std::runtime_error("failed to initialize serialized message");
  }

  auto copied_bytes = rcutils_snprintf(
    serialized_msg.buffer, initial_capacity, "%c%c%c%c%c%c%c%c%s\0",
    0x00, 0x01, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00, "Hello World!");
  serialized_msg.buffer_length = initial_capacity;
  for (auto i = 0u; i < serialized_msg.buffer_length; ++i) {
    fprintf(stderr, "%02x ", serialized_msg.buffer[i]);
  }
  fprintf(stderr, "\n");

  for (int i = 0u; i < 100; ++i) {
    fprintf(stderr, "publishing serialized data\n");
    ret = rcl_publish_serialized_message(&publisher, &serialized_msg);
    if (ret != RCL_RET_OK) {
      throw std::runtime_error("failed to publish serialized message");
    }
    sleep(1);
  }

  ret = rmw_serialized_message_fini(&serialized_msg);
  if (ret != RCL_RET_OK) {
    throw std::runtime_error("failed to cleanup serialized message");
  }
  return 0;
}
