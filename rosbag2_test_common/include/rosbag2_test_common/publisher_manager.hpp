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

#ifndef ROSBAG2_TEST_COMMON__PUBLISHER_MANAGER_HPP_
#define ROSBAG2_TEST_COMMON__PUBLISHER_MANAGER_HPP_

#include <functional>
#include <future>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"  // rclcpp must be included before the Windows specific includes.

using namespace std::chrono_literals;  // NOLINT

namespace rosbag2_test_common
{

using CountFunction = std::function<size_t(const std::string &)>;

class PublisherManager
{
public:
  ~PublisherManager()
  {
    publishers_.clear();
    publisher_nodes_.clear();
  }

  /**
   * Spin up a Node and publish message to topic_name message_count times at publish_rate.
   * The Node's scope is tied to the scope of this method.
   * The message may publish less than message_count times if rclcpp encountered an error.
   *
   * \tparam T the type of message to send.
   * \param topic_name is the name of the topic to publish to.
   * \param message is the message to publish.
   * \param publish_rate is the rate to publish the message
   * \param message_count is the number of times to publish the message.
   */
  template<class T>
  void run_scoped_publisher(
    const std::string & topic_name,
    const std::shared_ptr<T> message,
    const std::chrono::milliseconds publish_rate,
    const int message_count)
  {
    std::stringstream node_name;
    node_name << "publisher" << counter_++;

    auto publisher_node = std::make_shared<rclcpp::Node>(
      node_name.str(),
      rclcpp::NodeOptions().start_parameter_event_publisher(false));

    auto publisher = publisher_node->create_publisher<T>(topic_name, 10);
    rclcpp::WallRate rate{publish_rate};

    for (int i = 0; rclcpp::ok() && i < message_count; ++i) {
      publisher->publish(*message);
      rate.sleep();
    }
  }

  template<class T>
  void add_publisher(
    const std::string & topic_name,
    std::shared_ptr<T> message,
    size_t expected_messages = 0,
    const rclcpp::QoS & qos = rclcpp::QoS{rclcpp::KeepAll()})
  {
    auto node_name = std::string("publisher") + std::to_string(counter_++);
    auto publisher_node = std::make_shared<rclcpp::Node>(
      node_name,
      rclcpp::NodeOptions().start_parameter_event_publisher(false).enable_rosout(false));
    auto publisher = publisher_node->create_publisher<T>(topic_name, qos);

    publisher_nodes_.push_back(publisher_node);
    publishers_.push_back(
      [publisher, topic_name, message, expected_messages](
        CountFunction count_stored_messages) {
        if (expected_messages != 0) {
          while (rclcpp::ok() && count_stored_messages(topic_name) < expected_messages) {
            publisher->publish(*message);
            // rate limiting
            std::this_thread::sleep_for(50ms);
          }
        } else {
          // Just publish a few messages - they should never be stored
          publisher->publish(*message);
          std::this_thread::sleep_for(50ms);
          publisher->publish(*message);
          std::this_thread::sleep_for(50ms);
          publisher->publish(*message);
        }
      });
  }

  void run_publishers(CountFunction count_function)
  {
    std::vector<std::future<void>> futures;
    for (const auto & publisher : publishers_) {
      futures.push_back(std::async(std::launch::async, publisher, count_function));
    }
    for (auto & publisher_future : futures) {
      publisher_future.get();
    }
  }

private:
  int counter_ = 1;
  std::vector<std::shared_ptr<rclcpp::Node>> publisher_nodes_;
  std::vector<std::function<void(CountFunction)>> publishers_;
};

}  // namespace rosbag2_test_common

#endif  // ROSBAG2_TEST_COMMON__PUBLISHER_MANAGER_HPP_
