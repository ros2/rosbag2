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

#include <cstring>
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <utility>
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

    auto publisher_fcn = [publisher, message, repetition, interval](bool verbose = false) {
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
      };

    publishers_.push_back(std::make_pair(publisher, publisher_fcn));
  }

  void run_publishers(bool verbose = false) const
  {
    for (const auto & pub : publishers_) {
      std::get<PublicationF>(pub)(verbose);
    }
  }

  void run_publishers_async(bool verbose = false) const
  {
    std::vector<std::future<void>> futures;
    for (const auto & pub : publishers_) {
      futures.push_back(std::async(std::launch::async, std::get<PublicationF>(pub), verbose));
    }
    for (auto & publisher_future : futures) {
      publisher_future.get();
    }
  }

  /// \brief Wait until publisher with specified topic will be connected to the subscribers or
  ///    timeout occur.
  /// \tparam Timeout Data type for timeout duration from std::chrono:: namespace
  /// \param topic_name topic name to find corresponding publisher
  /// \param timeout Maximum time duration during which discovery should happen.
  /// \param n_subscribers_to_match Number of subscribers publisher should have for match.
  /// \return true if find publisher by specified topic_name and publisher has specified number of
  ///   subscribers, otherwise false.
  template<typename DurationRepT = int64_t, typename DurationT = std::milli>
  bool wait_for_matched(
    const char * topic_name,
    std::chrono::duration<DurationRepT, DurationT> timeout = std::chrono::seconds(10),
    size_t n_subscribers_to_match = 1)
  {
    rclcpp::PublisherBase::SharedPtr publisher = nullptr;
    for (const auto pub : publishers_) {
      if (!std::strcmp(
          std::get<rclcpp::PublisherBase::SharedPtr>(pub)->get_topic_name(),
          topic_name))
      {
        publisher = std::get<rclcpp::PublisherBase::SharedPtr>(pub);
        break;
      }
    }

    if (publisher == nullptr) {
      return false;
    }

    if (publisher->get_subscription_count() +
      publisher->get_intra_process_subscription_count() >= n_subscribers_to_match)
    {
      return true;
    }

    auto sleep_time = std::chrono::milliseconds(10);

    if (timeout < std::chrono::seconds(1)) {
      sleep_time = timeout;
    }

    using clock = std::chrono::steady_clock;
    auto start = clock::now();

    do {
      std::this_thread::sleep_for(sleep_time);

      if (publisher->get_subscription_count() +
        publisher->get_intra_process_subscription_count() >= n_subscribers_to_match)
      {
        return true;
      }
    } while ((clock::now() - start) < timeout);

    return false;
  }

private:
  std::shared_ptr<rclcpp::Node> pub_node_;
  using PublicationF = std::function<void (bool)>;
  std::vector<std::pair<rclcpp::PublisherBase::SharedPtr, PublicationF>> publishers_;
};

}  // namespace rosbag2_test_common

#endif  // ROSBAG2_TEST_COMMON__PUBLICATION_MANAGER_HPP_
