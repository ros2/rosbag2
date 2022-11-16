// Copyright 2020-2021, Robotec.ai sp. z o.o.
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

#include <exception>
#include <memory>
#include <string>
#include <vector>

#include "rosbag2_performance_benchmarking/byte_producer.hpp"
#include "rosbag2_performance_benchmarking/config_utils.hpp"
#include "rosbag2_performance_benchmarking/publisher_group_config.hpp"
#include "msg_utils/message_producer_factory.hpp"

#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/qos.hpp"
#include "rosbag2_performance_benchmarking_msgs/msg/byte_array.hpp"


class BenchmarkPublishers : public rclcpp::Node
{
public:
  explicit BenchmarkPublishers(const std::string & name)
  : rclcpp::Node(name)
  {
    create_benchmark_producers();
    wait_for_subscriptions();
  }

  void run()
  {
    std::vector<std::thread> producer_threads;
    for (auto & producer : producers_) {
      producer_threads.emplace_back(
        [this, producer]() {
          this->producer_job(producer);
        });
    }

    for (auto & thread : producer_threads) {
      thread.join();
    }
  }

private:
  struct BenchmarkProducer
  {
    std::shared_ptr<msg_utils::ProducerBase> producer;
    std::chrono::milliseconds period;
    size_t produced_messages = 0;
    size_t max_messages = 0;
  };

  void wait_for_subscriptions()
  {
    if (config_utils::wait_for_subscriptions_from_node_parameters(*this)) {
      for (auto producer : producers_) {
        producer->producer->wait_for_matched();
      }
    }
  }

  void create_benchmark_producers()
  {
    const auto configurations = config_utils::publisher_groups_from_node_parameters(*this);

    if (configurations.empty()) {
      RCLCPP_ERROR(get_logger(), "No publishers/producers found in node parameters");
      return;
    }

    const std::string node_name(get_fully_qualified_name());

    for (auto & config : configurations) {
      for (unsigned int i = 0; i < config.count; ++i) {
        const std::string topic = node_name + "/" + config.topic_root + "_" + std::to_string(i + 1);
        auto producer = create_benchmark_producer(topic, config);
        producers_.push_back(producer);
      }
    }
  }

  std::shared_ptr<BenchmarkProducer> create_benchmark_producer(
    std::string topic,
    const PublisherGroupConfig & config)
  {
    const auto & pConfig = config.producer_config;
    auto producer = std::make_shared<BenchmarkProducer>();

    producer->producer = msg_utils::create(pConfig.message_type, *this, topic, config);
    producer->max_messages = pConfig.max_count;
    producer->period = std::chrono::milliseconds(pConfig.frequency ? 1000 / pConfig.frequency : 1);

    return producer;
  }

  void producer_job(std::shared_ptr<BenchmarkProducer> producer)
  {
    for (auto i = 0u; i < producer->max_messages; ++i) {
      if (!rclcpp::ok()) {
        break;
      }
      std::this_thread::sleep_for(producer->period);
      producer->producer->produce();
      ++producer->produced_messages;
    }
  }

  std::vector<std::shared_ptr<BenchmarkProducer>> producers_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto publishers_node = std::make_shared<BenchmarkPublishers>(
    "rosbag2_performance_benchmarking_node");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(publishers_node);

  // The benchmark has its own control loop but uses spinning for parameters
  std::thread spin_thread([&executor]() {executor.spin();});
  publishers_node->run();
  RCLCPP_INFO(publishers_node->get_logger(), "Benchmark terminated");
  rclcpp::shutdown();
  spin_thread.join();

  return 0;
}
