// Copyright 2024 Open Source Robotics Foundation
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
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class SimpleBagPlayer : public rclcpp::Node
{
public:
  explicit SimpleBagPlayer(const std::string & bag_filename)
  : Node("playback_node")
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);

    timer_ = this->create_wall_timer(
      100ms,
      [this](){return this->timer_callback();}
    );

    reader_.open(bag_filename);
  }

private:
  void timer_callback()
  {
    while (reader_.has_next()) {
      rosbag2_storage::SerializedBagMessageSharedPtr msg = reader_.read_next();

      if (msg->topic_name != "chatter") {
        continue;
      }

      rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
      std_msgs::msg::String::SharedPtr ros_msg = std::make_shared<std_msgs::msg::String>();

      serialization_.deserialize_message(&serialized_msg, ros_msg.get());

      publisher_->publish(*ros_msg);
      std::cout << ros_msg->data << "\n";

      break;
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  rclcpp::Serialization<std_msgs::msg::String> serialization_;
  rosbag2_cpp::Reader reader_;
};

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <bag>" << std::endl;
    return 1;
  }

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleBagPlayer>(argv[1]));
  rclcpp::shutdown();

  return 0;
}
