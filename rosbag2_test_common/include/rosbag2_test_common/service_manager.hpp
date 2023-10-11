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

namespace rosbag2_test_common
{
class ServiceManager
{
public:
  ServiceManager()
  : pub_node_(std::make_shared<rclcpp::Node>(
        "service_manager_" + std::to_string(rclcpp::Clock().now().nanoseconds()),
        rclcpp::NodeOptions().start_parameter_event_publisher(false).enable_rosout(false)))
  {
  }

  ~ServiceManager()
  {
    exit_ = true;
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

    auto service = pub_node_->create_service<ServiceT>(
      service_name, std::forward<decltype(callback)>(callback));
    services_.emplace(service_name, service);
  }

  void run_services()
  {
    auto spin = [this]() {
        rclcpp::executors::SingleThreadedExecutor exec;
        exec.add_node(pub_node_);

        while (!exit_) {
          exec.spin_some();
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
      };

    thread_ = std::thread(spin);
  }

private:
  std::shared_ptr<rclcpp::Node> pub_node_;
  std::unordered_map<std::string, typename rclcpp::ServiceBase::SharedPtr> services_;
  bool exit_ = false;
  std::thread thread_;
};

}  // namespace rosbag2_test_common

#endif  // ROSBAG2_TEST_COMMON__SERVICE_MANAGER_HPP_
