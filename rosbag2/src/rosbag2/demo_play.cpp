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

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rosbag2/rosbag2_play_options.hpp"
#include "rosbag2/rosbag2.hpp"

int main(int argc, const char ** argv)
{
  rclcpp::init(argc, argv);

  auto options = rosbag2::Rosbag2PlayOptions();
  options.read_ahead_queue_size = 1000;

  rosbag2::Rosbag2 rosbag2;
  rosbag2.play("test.bag", options);

  rclcpp::shutdown();

  return 0;
}
