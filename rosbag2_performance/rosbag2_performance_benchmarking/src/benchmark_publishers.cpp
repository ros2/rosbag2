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

#include "rosbag2_performance_benchmarking/byte_producer.hpp"

#include "std_msgs/msg/byte_multi_array.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/qos.hpp"

#include <vector>

struct PublisherGroupConfiguration
{
  PublisherGroupConfiguration() : count(0), qos(10) {}
  uint count;
  ProducerConfig producer_config;
  std::string topic_root;
  rclcpp::QoS qos;
};

class BenchmarkPublishers : public rclcpp::Node
{
public:
  explicit BenchmarkPublishers(const std::string & name)
    : rclcpp::Node(name)
  {
    process_configuration();
    create_benchmark_publishers_and_producers();
  }

  void run()
  {
    std::vector<std::thread> producer_threads;
    for (auto & producer : producers_) {
      producer_threads.push_back(std::thread(&ByteProducer::run, producer.get()));
    }

    for (auto & thread : producer_threads) {
      thread.join();
    }
  }

private:
  void process_configuration()
  {
    std::vector<std::string> publisher_groups;
    this->declare_parameter("publishers.publisher_groups");
    this->get_parameter("publishers.publisher_groups", publisher_groups);
    for (const auto & group_name : publisher_groups) {
      auto group_prefix = "publishers." + group_name;
      this->declare_parameter(group_prefix + ".publishers_count");
      this->declare_parameter(group_prefix + ".topic_root");
      this->declare_parameter(group_prefix + ".msg_size_bytes");
      this->declare_parameter(group_prefix + ".msg_count_each");
      this->declare_parameter(group_prefix + ".rate_hz");

      PublisherGroupConfiguration group_config;
      this->get_parameter(group_prefix + ".publishers_count", group_config.count);
      this->get_parameter(group_prefix + ".topic_root", group_config.topic_root);
      this->get_parameter(group_prefix + ".msg_size_bytes", group_config.producer_config.message_size);
      this->get_parameter(group_prefix + ".msg_count_each", group_config.producer_config.max_count);
      this->get_parameter(group_prefix + ".rate_hz", group_config.producer_config.frequency);
      process_qos_configuration(group_config, group_prefix);

      configurations_.push_back(group_config);
    }
  }

  void process_qos_configuration(
    PublisherGroupConfiguration & group_config,
    const std::string & group_prefix)
  {
    auto qos_prefix = group_prefix + ".qos";
    this->declare_parameter(qos_prefix + ".qos_depth");
    this->declare_parameter(qos_prefix + ".qos_reliability");
    this->declare_parameter(qos_prefix + ".qos_durability");

    uint qos_depth = 10;
    std::string qos_reliability, qos_durability;
    this->get_parameter(qos_prefix + ".qos_depth", qos_depth);
    this->get_parameter(qos_prefix + ".qos_reliability", qos_reliability);
    this->get_parameter(qos_prefix + ".qos_durability", qos_durability);

    group_config.qos.keep_last(qos_depth);
    // TODO(adamdbrw) - error handling / map string to function
    if (qos_reliability == "reliable") group_config.qos.reliable();
    if (qos_reliability == "best_effort") group_config.qos.best_effort();
    if (qos_reliability == "transient_local") group_config.qos.transient_local();
    if (qos_reliability == "volatile") group_config.qos.durability_volatile();
  }

  void create_benchmark_publishers_and_producers()
  {
    const std::string topic_prefix(this->get_fully_qualified_name());
    for (auto & c : configurations_) {
      for (uint i = 0; i < c.count; ++i) {
        std::string topic = topic_prefix + "/" + c.topic_root + "_" + std::to_string(i+1);
        auto pub = this->create_publisher<std_msgs::msg::ByteMultiArray>(topic, c.qos);
        publishers_.push_back(pub);
        producers_.push_back(std::make_unique<ByteProducer>(
          c.producer_config,
          [pub] (std::shared_ptr<std_msgs::msg::ByteMultiArray> msg) {
            pub->publish(*msg);
          },
          [] {  /* empty lambda */ }));
      }
    }
  }

  std::vector<PublisherGroupConfiguration> configurations_;
  std::vector<std::unique_ptr<ByteProducer>> producers_;
  std::vector<std::shared_ptr<rclcpp::Publisher<std_msgs::msg::ByteMultiArray>>> publishers_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto publishers_node = std::make_shared<BenchmarkPublishers>(
    "rosbag2_performance_benchmarking_publishers");

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
