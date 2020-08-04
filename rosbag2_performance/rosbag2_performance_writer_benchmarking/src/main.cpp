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

#include <memory>
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rosbag2_performance_writer_benchmarking/writer_benchmark.hpp"

// TODO(adamdbrw) the benchmkark being ROS node is not necessary, only used for logging
// and parameters. Otherwise ROS dependencies are calls to rcl and rmw.
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto bench = std::make_shared<WriterBenchmark>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(bench);

  // The benchmark has its own control loop but uses spinning for parameters
  std::thread spin_thread([&executor]() {executor.spin();});
  bench->start_benchmark();
  RCLCPP_INFO(bench->get_logger(), "Benchmark terminated");
  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}
