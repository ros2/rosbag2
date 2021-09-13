// Copyright 2021 Open Source Robotics Foundation
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

#include <chrono>
#include <memory>

#include "example_interfaces/msg/int32.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rosbag2_cpp/writer.hpp"

using namespace std::chrono_literals;

class DataGenerator : public rclcpp::Node
{
public:
  DataGenerator()
  : Node("data_generator")
  {
    data.data = 0;
    writer_ = std::make_unique<rosbag2_cpp::Writer>();

    writer_->open("timed_synthetic_bag");

    writer_->create_topic(
    {
      "synthetic",
      "example_interfaces/msg/Int32",
      rmw_get_serialization_format(),
      ""
    });

    timer_ = create_wall_timer(1s, std::bind(&DataGenerator::timer_callback, this));
  }

private:
  void timer_callback()
  {
    writer_->write(data, "synthetic", now());

    ++data.data;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  example_interfaces::msg::Int32 data;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DataGenerator>());
  rclcpp::shutdown();
  return 0;
}
