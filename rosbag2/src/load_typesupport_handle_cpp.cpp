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

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "rosidl_generator_c/message_type_support_struct.h"

#include "utils/serialized_message_utils.hpp"
#include "utils/typesupport_utils.hpp"

int main(int argc, char ** argv)
{
  std::string package_name = "std_msgs";
  std::string message_name = "String";

  auto ts = utils::get_typesupport_cpp(package_name, message_name);
  if (ts == nullptr) {
    fprintf(stderr, "unable to get typesupport");
    return -1;
  }

  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("load_typesupport_cpp");
  auto publisher_base = std::make_shared<rclcpp::PublisherBase>(
    node->get_node_base_interface().get(),
    "dynamic_message",
    *ts,
    rcl_publisher_get_default_options());


  auto serialized_msg = utils::create_binary_string_message();
  for (int i = 0u; i < 100; ++i) {
    fprintf(stderr, "publishing serialized data\n");
    auto ret = rcl_publish_serialized_message(
      publisher_base->get_publisher_handle(), &serialized_msg);
    if (ret != RCL_RET_OK) {
      throw std::runtime_error("failed to publish serialized message");
    }
    sleep(1);
  }
  utils::destroy_binary_string_message(&serialized_msg);

  return 0;
}
