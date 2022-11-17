// Copyright 2022 Apex.AI, Inc.
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

#ifndef MSG_UTILS__MESSAGE_PRODUCER_HPP_
#define MSG_UTILS__MESSAGE_PRODUCER_HPP_

#include <memory>
#include <string>
#include <stdexcept>

#include "helpers.hpp"
#include "rclcpp/node.hpp"
#include "rosbag2_performance_benchmarking/publisher_group_config.hpp"

namespace msg_utils
{
class ProducerBase
{
public:
  virtual void produce() = 0;
  virtual void wait_for_matched() = 0;
};

template<typename T>
class MessageProducer : public ProducerBase
{
public:
  MessageProducer(rclcpp::Node & node, std::string topic, const PublisherGroupConfig & config);

  void produce() override;
  void wait_for_matched() override;

private:
  void resize(size_t size);

  T message_;
  std::shared_ptr<rclcpp::Publisher<T>> publisher_;
};

template<typename T>
MessageProducer<T>::MessageProducer(
  rclcpp::Node & node, std::string topic,
  const PublisherGroupConfig & config)
: publisher_(node.create_publisher<T>(topic, config.qos))
{
  if (config.producer_config.message_size > 0) {
    resize(config.producer_config.message_size);
  }
}

template<typename T>
void MessageProducer<T>::wait_for_matched()
{
  const double max_subscription_wait_time = 5.0;
  auto start_time = std::chrono::high_resolution_clock::now();
  while (publisher_->get_subscription_count() == 0U) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    auto current_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = current_time - start_time;
    if (elapsed.count() >= max_subscription_wait_time) {
      throw std::runtime_error("Waited too long for subscription");
    }
  }
}

template<typename T>
void MessageProducer<T>::produce()
{
  if (!rclcpp::ok()) {
    return;
  }
  publisher_->publish(message_);
}

template<typename T>
void MessageProducer<T>::resize(size_t size)
{
  (void)size;
  throw std::runtime_error(
          "Resize not implemented for type: " +
          std::string(rosidl_generator_traits::data_type<T>()));
}

template<>
void MessageProducer<std_msgs::msg::ByteMultiArray>::resize(size_t size)
{
  helpers::generate_data(message_, size);
}

template<>
void MessageProducer<sensor_msgs::msg::Image>::resize(size_t size)
{
  helpers::generate_data(message_, size);
}

template<>
void MessageProducer<sensor_msgs::msg::PointCloud2>::resize(size_t size)
{
  helpers::generate_data(message_, size);
}

}  // namespace msg_utils

#endif  // MSG_UTILS__MESSAGE_PRODUCER_HPP_
