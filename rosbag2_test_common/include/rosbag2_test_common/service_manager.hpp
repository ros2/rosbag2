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

#ifndef ROSBAG2_TEST_COMMON__SERVICE_MANAGER_HPP_
#define ROSBAG2_TEST_COMMON__SERVICE_MANAGER_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "rosbag2_test_common/wait_for.hpp"

namespace rosbag2_test_common
{
class ServiceManager
{
public:
  ServiceManager()
  : service_node_(std::make_shared<rclcpp::Node>(
        "service_manager_" + std::to_string(rclcpp::Clock().now().nanoseconds()),
        rclcpp::NodeOptions()
        .start_parameter_event_publisher(false)
        .enable_rosout(false)
        .start_parameter_services(false))),
    check_service_ready_node_(std::make_shared<rclcpp::Node>(
        "check_service_ready_node_" + std::to_string(rclcpp::Clock().now().nanoseconds()),
        rclcpp::NodeOptions()
        .start_parameter_event_publisher(false)
        .enable_rosout(false)
        .start_parameter_services(false)))
  {
  }

  ~ServiceManager()
  {
    exec_.cancel();
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  template<typename ServiceT>
  void setup_service(
    std::string service_name,
    std::vector<std::shared_ptr<typename ServiceT::Request>> & requests)
  {
    auto callback = [&requests](
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<typename ServiceT::Request> request,
      std::shared_ptr<typename ServiceT::Response> response) {
        (void)request_header;
        (void)response;
        requests.emplace_back(request);
      };

    auto service = service_node_->create_service<ServiceT>(
      service_name, std::forward<decltype(callback)>(callback));
    service->configure_introspection(
      service_node_->get_clock(), rclcpp::ServicesQoS(), RCL_SERVICE_INTROSPECTION_CONTENTS);
    services_.emplace(service_name, service);

    auto client = check_service_ready_node_->create_client<ServiceT>(service_name);
    client->configure_introspection(
      service_node_->get_clock(), rclcpp::ServicesQoS(), RCL_SERVICE_INTROSPECTION_CONTENTS);
    clients_.emplace_back(client);
  }

  void run_services()
  {
    exec_.add_node(service_node_);
    thread_ = std::thread(
      [this]() {
        exec_.spin();
      });

    // Wait for the executor to start spinning in the newly spawned thread to avoid race conditions
    if (!wait_until_condition([this]() {return exec_.is_spinning();}, std::chrono::seconds(5))) {
      std::cerr << "Failed to start spinning node:" << service_node_->get_name() << std::endl;
      throw std::runtime_error("Failed to start spinning node");
    }
  }

  bool all_services_ready()
  {
    for (auto client : clients_) {
      if (!client->wait_for_service(std::chrono::seconds(2))) {
        return false;
      }
    }
    return true;
  }

private:
  std::shared_ptr<rclcpp::Node> service_node_;
  std::shared_ptr<rclcpp::Node> check_service_ready_node_;
  std::unordered_map<std::string, typename rclcpp::ServiceBase::SharedPtr> services_;
  std::vector<typename rclcpp::ClientBase::SharedPtr> clients_;
  std::thread thread_;
  rclcpp::executors::SingleThreadedExecutor exec_;
};

}  // namespace rosbag2_test_common

#endif  // ROSBAG2_TEST_COMMON__SERVICE_MANAGER_HPP_
