// Copyright 2021, Robotec.ai sp. z o.o.
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

#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/node.hpp"
#include "rosbag2_performance_benchmarking/result_utils.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto results_writer_node = std::make_shared<rclcpp::Node>(
    "benchmarking_results_writer");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(results_writer_node);

  // The benchmark uses spinning for parameters
  std::thread spin_thread([&executor]() {executor.spin();});
  result_utils::write_benchmark_results(*results_writer_node);
  rclcpp::shutdown();
  spin_thread.join();

  return 0;
}
