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

#include <memory>
#include <string>
#include <vector>

#include "rosbag2_performance_benchmarking/byte_producer.hpp"
#include "rosbag2_performance_benchmarking/config_utils.hpp"
#include "rosbag2_performance_benchmarking/publisher_group_config.hpp"

#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/qos.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"


class BenchmarkPublishers : public rclcpp::Node
{
public:
  explicit BenchmarkPublishers(const std::string & name)
  : rclcpp::Node(name)
  {
    configurations_ = config_utils::publisher_groups_from_node_parameters(*this);
    if (configurations_.empty()) {
      RCLCPP_ERROR(get_logger(), "No publishers/producers found in node parameters");
      return;
    }

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
  void create_benchmark_publishers_and_producers()
  {
    const std::string topic_prefix(this->get_fully_qualified_name());
    for (auto & c : configurations_) {
      for (uint i = 0; i < c.count; ++i) {
        auto topic = topic_prefix + "/" + c.topic_root + "_" + std::to_string(i + 1);
        auto pub = this->create_publisher<std_msgs::msg::ByteMultiArray>(topic, c.qos);
        publishers_.push_back(pub);
        producers_.push_back(
          std::make_unique<ByteProducer>(
            c.producer_config,
            [pub](std::shared_ptr<std_msgs::msg::ByteMultiArray> msg) {
              pub->publish(*msg);
            },
            [] { /* empty lambda */}));
      }
    }
  }

  std::vector<PublisherGroupConfig> configurations_;
  std::vector<std::unique_ptr<ByteProducer>> producers_;
  std::vector<std::shared_ptr<rclcpp::Publisher<std_msgs::msg::ByteMultiArray>>> publishers_;
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
