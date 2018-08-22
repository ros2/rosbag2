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

#include <cstdio>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "rosbag2/rosbag2.hpp"

int main(int argc, const char ** argv)
{
  if (argc < 2) {
    std::cerr << "\nThe name of the topic to record must be given as parameter!\n";
    return 0;
  }
  std::string topic_name = argv[1];

  // TODO(anhosi): allow output file to be specified by cli argument and do proper checking if
  // file already exists
  std::string filename("test.bag");
  std::remove(filename.c_str());

  rclcpp::init(argc, argv);

  rosbag2::Rosbag2 rosbag2;
  rosbag2.record(filename, topic_name);

  rclcpp::shutdown();

  return 0;
}
