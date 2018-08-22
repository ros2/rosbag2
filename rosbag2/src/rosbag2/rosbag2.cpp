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

#include "rosbag2/rosbag2.hpp"

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "rcutils/logging_macros.h"

#include "std_msgs/msg/string.hpp"

#include "rosbag2_storage/serialized_bag_message.hpp"
#include "rosbag2_storage/storage_interfaces/read_only_interface.hpp"
#include "rosbag2_storage/storage_interfaces/read_write_interface.hpp"
#include "rosbag2_storage/storage_factory.hpp"
#include "demo_helpers.hpp"

namespace rosbag2
{

const char * ROS_PACKAGE_NAME = "rosbag2";

void Rosbag2::record(
  const std::string & file_name,
  const std::string & topic_name,
  std::function<void(void)> after_write_action)
{
  (void) after_write_action;
  rosbag2_storage::StorageFactory factory;
  auto storage = factory.open_read_write(file_name, "sqlite3");

  if (storage) {
    std::string type_name = "String";
    std::string package_name = "std_msgs";

    auto library_path = get_typesupport_library(package_name);
    std::shared_ptr<Poco::SharedLibrary> typesupport_library = nullptr;
    const rosidl_message_type_support_t * ts = nullptr;

    try {
      typesupport_library = std::make_shared<Poco::SharedLibrary>(library_path);

      auto symbol_name = std::string("rosidl_typesupport_c__get_message_type_support_handle__")
        + package_name + "__msg__" + type_name;

      if (!typesupport_library->hasSymbol(symbol_name)) {
        std::cout << "\nerror 1\n";
        return;
      }

      const rosidl_message_type_support_t * (*get_ts)(void) = nullptr;
      get_ts = (decltype(get_ts)) typesupport_library->getSymbol(symbol_name);
      ts = get_ts();
      if (!ts) {
        std::cout << "\nerror 2\n";
        return;
      }
    } catch (Poco::LibraryLoadException &) {
      std::cout << "\nerror 3\n";
    }

    auto ret = RCL_RET_ERROR;

    auto node_ptr = new rcl_node_t;
    *node_ptr = rcl_get_zero_initialized_node();
    const char * name = "rosbag2_node";
    rcl_node_options_t node_options = rcl_node_get_default_options();
    ret = rcl_node_init(node_ptr, name, "", &node_options);

    auto subscription = rcl_get_zero_initialized_subscription();
    auto subscription_options = rcl_subscription_get_default_options();
    ret =  rcl_subscription_init(
      &subscription, node_ptr, ts, topic_name.c_str(), &subscription_options);

    ret = RCL_RET_SUBSCRIPTION_TAKE_FAILED;
    RCUTILS_LOG_INFO_NAMED(ROS_PACKAGE_NAME, "Waiting for messages...");

    while ((ret == RCL_RET_OK || ret == RCL_RET_SUBSCRIPTION_TAKE_FAILED) && rclcpp::ok()) {
      auto serialized_msg = new rcutils_char_array_t;
      *serialized_msg = rcutils_get_zero_initialized_char_array();
      auto allocator = new rcutils_allocator_t;
      *allocator = rcutils_get_default_allocator();
      ret = rcutils_char_array_init(
        serialized_msg,
        0,
        allocator);
      if (ret != RCL_RET_OK) {
        throw std::runtime_error("failed to initialize serialized message");
      }

      ret = rcl_take_serialized_message(&subscription, serialized_msg, nullptr);

      if (ret == RCL_RET_OK) {
        auto message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
        message->serialized_data = std::shared_ptr<rcutils_char_array_t>(serialized_msg,
          [](rcutils_char_array_t * msg) {
            int error = rcutils_char_array_fini(msg);
            delete msg;
            if (error != RCUTILS_RET_OK) {
              RCUTILS_LOG_ERROR_NAMED(
                "rosbag2_storage_default_plugins",
                "Leaking memory. Error: %s", rcutils_get_error_string_safe());
            }
          });
        message->topic_name = topic_name;
        rcutils_time_point_value_t time_stamp;
        int error = rcutils_system_time_now(&time_stamp);
        if (error != RCUTILS_RET_OK) {
          RCUTILS_LOG_ERROR_NAMED(
            "rosbag2", "Error getting current time. Error: %s", rcutils_get_error_string_safe());
        }
        message->time_stamp = time_stamp;
        storage->create_topic(topic_name, ts->typesupport_identifier);
        storage->write(message);
        if (after_write_action) {
          after_write_action();
        }
      } else {
        (void) rcutils_char_array_fini(serialized_msg);
      }
    }
    (void) rcl_subscription_fini(&subscription, node_ptr);
    (void) rcl_node_fini(node_ptr);
  }
}

void Rosbag2::play(const std::string & file_name, const std::string & topic_name)
{
  rosbag2_storage::StorageFactory factory;
  auto storage = factory.open_read_only(file_name, "sqlite3");

  if (storage) {
    auto node = std::make_shared<rclcpp::Node>("rosbag_publisher_node");
    auto publisher = node->create_publisher<std_msgs::msg::String>(topic_name);
    while (storage->has_next()) {
      auto message = storage->read_next();

      // without the sleep_for() many messages are lost.
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      publisher->publish(message->serialized_data.get());
      RCUTILS_LOG_INFO_NAMED(ROS_PACKAGE_NAME, "published message");
    }
  }
}

}  // namespace rosbag2
