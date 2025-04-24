// Copyright 2023 Sony Group Corporation.
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

#ifndef ROSBAG2_TEST_COMMON__CLIENT_MANAGER_HPP_
#define ROSBAG2_TEST_COMMON__CLIENT_MANAGER_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rcl/service_introspection.h"

#include "rclcpp/rclcpp.hpp"  // rclcpp must be included before the Windows specific includes.

namespace rosbag2_test_common
{
template<typename ServiceT>
class ClientManager : public rclcpp::Node
{
public:
  explicit ClientManager(
    std::string service_name,
    size_t number_of_clients = 1,
    bool service_event_contents = false,
    bool client_event_contents = true)
  : Node("service_client_manager_" + std::to_string(rclcpp::Clock().now().nanoseconds()),
      rclcpp::NodeOptions().start_parameter_services(false).start_parameter_event_publisher(
        false).enable_rosout(false)),
    service_name_(std::move(service_name)),
    number_of_clients_(number_of_clients),
    enable_service_event_contents_(service_event_contents),
    enable_client_event_contents_(client_event_contents)
  {
    // *INDENT-OFF*
    auto do_nothing_srv_callback =
      [this](const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<typename ServiceT::Request> request,
        std::shared_ptr<typename ServiceT::Response> response) -> void
      {
        // Do nothing
        (void)request_header;
        (void)request;
        (void)response;
      };
    // *INDENT-ON*

    service_ = create_service<ServiceT>(service_name_, do_nothing_srv_callback);

    rcl_service_introspection_state_t introspection_state;
    if (enable_service_event_contents_) {
      introspection_state = RCL_SERVICE_INTROSPECTION_CONTENTS;
    } else {
      introspection_state = RCL_SERVICE_INTROSPECTION_OFF;
    }
    service_->configure_introspection(
      get_clock(), rclcpp::ServicesQoS(), introspection_state);

    if (enable_client_event_contents_) {
      introspection_state = RCL_SERVICE_INTROSPECTION_CONTENTS;
    } else {
      introspection_state = RCL_SERVICE_INTROSPECTION_OFF;
    }

    for (size_t i = 0; i < number_of_clients_; i++) {
      auto client = create_client<ServiceT>(service_name_);
      client->configure_introspection(
        get_clock(), rclcpp::ServicesQoS(), introspection_state);
      clients_.emplace_back(client);
    }
  }

  bool check_service_ready()
  {
    for (auto & client : clients_) {
      if (!client->service_is_ready()) {
        return false;
      }
    }
    return true;
  }

  bool wait_for_service_to_be_ready(std::chrono::duration<double> timeout = std::chrono::seconds(5))
  {
    using clock = std::chrono::system_clock;
    auto start = clock::now();
    while (!check_service_ready() && (clock::now() - start) < timeout) {
      std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
    return check_service_ready();
  }

  bool send_request(std::chrono::duration<double> timeout = std::chrono::seconds(5))
  {
    if (!check_service_ready()) {
      return false;
    }

    for (auto & client : clients_) {
      auto request = std::make_shared<typename ServiceT::Request>();
      auto result = client->async_send_request(request);
      // Wait for the result.
      if (rclcpp::executors::spin_node_until_future_complete(
          exec_, get_node_base_interface(), result, timeout) != rclcpp::FutureReturnCode::SUCCESS)
      {
        RCLCPP_INFO(this->get_logger(), "Failed to get response !");
        return false;
      }
    }
    return true;
  }

  using client_shared_ptr = typename rclcpp::Client<ServiceT>::SharedPtr;

private:
  rclcpp::executors::SingleThreadedExecutor exec_;
  typename rclcpp::Service<ServiceT>::SharedPtr service_;
  std::vector<client_shared_ptr> clients_;
  const std::string service_name_;
  size_t number_of_clients_;
  bool enable_service_event_contents_;
  bool enable_client_event_contents_;
};
}  // namespace rosbag2_test_common

#endif  // ROSBAG2_TEST_COMMON__CLIENT_MANAGER_HPP_
