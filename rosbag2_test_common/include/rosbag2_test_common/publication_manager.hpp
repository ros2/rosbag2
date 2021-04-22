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

#ifndef ROSBAG2_TEST_COMMON__PUBLICATION_MANAGER_HPP_
#define ROSBAG2_TEST_COMMON__PUBLICATION_MANAGER_HPP_

#include <functional>
#include <future>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"  // rclcpp must be included before the Windows specific includes.

using namespace std::chrono_literals;  // NOLINT

namespace rosbag2_test_common
{

class PublicationManager
{
public:
  PublicationManager()
  : pub_node_(std::make_shared<rclcpp::Node>(
        "publication_manager_" + std::to_string(rclcpp::Clock().now().nanoseconds()),
        rclcpp::NodeOptions().start_parameter_event_publisher(false).enable_rosout(false)))
  {}

  template<class MsgT>
  void setup_publisher(
    const std::string & topic_name,
    const std::shared_ptr<MsgT> message,
    const size_t repetition,
    const rclcpp::QoS & qos = rclcpp::QoS{rclcpp::KeepAll()},
    const std::chrono::milliseconds interval = 50ms)
  {
    return setup_publisher(topic_name, *message, repetition, qos, interval);
  }

  template<class MsgT>
  void setup_publisher(
    const std::string & topic_name,
    const MsgT & message,
    const size_t repetition,
    const rclcpp::QoS & qos = rclcpp::QoS{rclcpp::KeepAll()},
    const std::chrono::milliseconds interval = 50ms)
  {
    auto publisher = pub_node_->create_publisher<MsgT>(topic_name, qos);

    publishers_.push_back(
      [publisher, message, repetition, interval](bool verbose = false) {
        for (auto i = 0u; i < repetition; ++i) {
          if (rclcpp::ok()) {
            publisher->publish(message);
            if (verbose) {
              RCLCPP_INFO(
                rclcpp::get_logger("publication_manager"),
                "publish on topic %s", publisher->get_topic_name());
            }
            std::this_thread::sleep_for(interval);
          }
        }
      });
  }

  void run_publishers(bool verbose = false) const
  {
    for (const auto & pub_fcn : publishers_) {
      pub_fcn(verbose);
    }
  }

  void run_publishers_async(bool verbose = false) const
  {
    std::vector<std::future<void>> futures;
    for (const auto & pub_fcn : publishers_) {
      futures.push_back(std::async(std::launch::async, pub_fcn, verbose));
    }
    for (auto & publisher_future : futures) {
      publisher_future.get();
    }
  }

private:
  std::shared_ptr<rclcpp::Node> pub_node_;
  using PublicationF = std::function<void (bool)>;
  std::vector<PublicationF> publishers_;
};

}  // namespace rosbag2_test_common

#endif  // ROSBAG2_TEST_COMMON__PUBLICATION_MANAGER_HPP_
