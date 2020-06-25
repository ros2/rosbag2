// Copyright 2020, Robotec.ai sp. z o.o.
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

#ifndef ROSBAG2_PERFORMANCE_WORKERS_HPP
#define ROSBAG2_PERFORMANCE_WORKERS_HPP

#include <chrono>
#include <string>
#include <vector>
#include <memory>

#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

template<typename T>
class Worker : public rclcpp::Node
{
  typedef typename rclcpp::Publisher<T>::SharedPtr publisherPtr;

public:
  Worker(const std::string & name)
  : Node(name)
  {
    this->declare_parameter("frequency");
    this->declare_parameter("max_count");
    this->declare_parameter("size");
    this->declare_parameter("delay");
    this->declare_parameter("benchmark_path");
    this->declare_parameter("instances");
    this->declare_parameter("topic");
    this->declare_parameter("same_topic");

    auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this);
    while (!parameters_client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        rclcpp::shutdown();
      }
      RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }
    frequency = parameters_client->get_parameter("frequency", 100);
    if (frequency == 0) {
      RCLCPP_ERROR(this->get_logger(), "Frequency can't be 0. Exiting.");
      rclcpp::shutdown(nullptr, "frequency error");
      return;
    }

    max_count = parameters_client->get_parameter("max_count", 100);
    size = parameters_client->get_parameter("size", 1024);
    delay = parameters_client->get_parameter("delay", 0);
    instances = parameters_client->get_parameter("instances", 1);
    benchmark_path =
      parameters_client->get_parameter(std::string("benchmark_path"), std::string(""));
    topic = parameters_client->get_parameter(std::string("topic"), std::string("worker_topic"));
    same_topic = parameters_client->get_parameter("same_topic", true);

    for (uint32_t i = 0; i < instances; ++i) {
      auto suffix = same_topic ? "" : std::to_string(i);
      publishers.push_back(this->create_publisher<T>(topic + suffix, 10));
    }
    delay_timer =
      this->create_wall_timer(
      std::chrono::milliseconds(delay),
      std::bind(&Worker::delay_callback, this));
  }

  virtual T getMessage(const uint32_t & size) = 0;

protected:
  std::string benchmark_path;
  std::string topic;
  bool same_topic;
  uint32_t frequency;
  uint32_t max_count;
  uint32_t size;
  uint32_t delay;
  uint32_t instances;

private:
  void delay_callback() noexcept
  {
    std::cout << this->get_name() << ": Delay " << delay << " finished" << std::endl;
    timer =
      this->create_wall_timer(
      std::chrono::milliseconds(1000 / frequency),
      std::bind(&Worker::timer_callback, this));
    delay_timer->cancel();
  }

  void timer_callback() noexcept
  {
    uint32_t static current_msg_count = 0;

    auto message = getMessage(size);
    for (uint32_t i = 0; i < instances; ++i) {
      publishers[i]->publish(message);
    }

    std::cout << this->get_name() << ": " << current_msg_count << std::endl;
    if (++current_msg_count == max_count) {
      timer->cancel();
      rclcpp::shutdown();
    }

    //(piotr.jaroszek) TODO: raport data here and save to benchmark_path
  }

  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::TimerBase::SharedPtr delay_timer;
  std::vector<publisherPtr> publishers;
};

#endif //ROSBAG2_PERFORMANCE_WORKERS_HPP
