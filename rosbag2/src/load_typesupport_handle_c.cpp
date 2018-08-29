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

#include <string>

#include "utils/serialized_message_utils.hpp"
#include "utils/typesupport_utils.hpp"

#include "rcl/rcl.h"

#include "rosidl_generator_c/message_type_support_struct.h"

int main(int argc, char ** argv)
{
  std::string package_name = "std_msgs";
  std::string message_name = "String";

  auto ts = utils::get_typesupport_c(package_name, message_name);
  if (ts == nullptr) {
    fprintf(stderr, "unable to get typesupport");
    return -1;
  }

  auto ret = RCL_RET_ERROR;
  ret = rcl_init(argc, argv, rcl_get_default_allocator());
  auto node_ptr = new rcl_node_t;
  *node_ptr = rcl_get_zero_initialized_node();
  const char * name = "load_typesupport_c";
  rcl_node_options_t node_options = rcl_node_get_default_options();
  ret = rcl_node_init(node_ptr, name, "", &node_options);

  rcl_publisher_t publisher = rcl_get_zero_initialized_publisher();
  rcl_publisher_options_t publisher_options = rcl_publisher_get_default_options();
  ret = rcl_publisher_init(&publisher, node_ptr, ts, "dynamic_message", &publisher_options);

  auto serialized_msg = utils::create_binary_string_message();
  for (int i = 0u; i < 100; ++i) {
    fprintf(stderr, "publishing serialized data\n");
    ret = rcl_publish_serialized_message(&publisher, &serialized_msg);
    if (ret != RCL_RET_OK) {
      throw std::runtime_error("failed to publish serialized message");
    }
    sleep(1);
  }
  utils::destroy_binary_string_message(&serialized_msg);

  return 0;
}
