// Copyright 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#ifndef ROSBAG2_TEST_COMMON__WAIT_FOR_HPP_
#define ROSBAG2_TEST_COMMON__WAIT_FOR_HPP_

#include <chrono>

#include "rclcpp/rclcpp.hpp"

namespace rosbag2_test_common
{
template<typename Timeout, typename Node, typename Condition>
bool spin_and_wait_for(Timeout timeout, const Node & node, Condition condition)
{
  using clock = std::chrono::system_clock;
  auto start = clock::now();
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  while (!condition()) {
    if ((clock::now() - start) > timeout) {
      return false;
    }
    exec.spin_some();
  }
  return true;
}

template<typename Timeout, typename Condition>
bool wait_until_shutdown(Timeout timeout, Condition condition)
{
  using clock = std::chrono::system_clock;
  auto start = clock::now();
  while (!condition()) {
    if ((clock::now() - start) > timeout) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  rclcpp::shutdown();
  return true;
}
}  // namespace rosbag2_test_common

#endif  // ROSBAG2_TEST_COMMON__WAIT_FOR_HPP_
