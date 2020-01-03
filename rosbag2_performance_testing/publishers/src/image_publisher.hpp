// Copyright 2019, Martin Idel
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
#include <string>
#include <vector>

#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

using namespace std::chrono_literals;  // NOLINT

std::vector<uint8_t> randomImageData(size_t size) {
  std::vector<uint8_t> pixels(size, 0);

  for (size_t i = 0; i < size; ++i) {
    pixels[i] = std::rand() % 255;
  }
  return pixels;
}

namespace nodes {

class ImagePublisher : public rclcpp::Node {
public:
  ImagePublisher(const std::string & node_name, const std::string & topic_name)
  : Node(node_name)
  {
    publisher = this->create_publisher<sensor_msgs::msg::Image>(topic_name, 10);
    timer = this->create_wall_timer(100ms, std::bind(&ImagePublisher::timer_callback, this));
    random_image_data_ = randomImageData(750 * 4 * 750);  // image step * height
  }

private:
  void timer_callback()
  {
    auto message = sensor_msgs::msg::Image();
    message.header = std_msgs::msg::Header();
    message.header.frame_id = "image_frame";
    message.header.stamp = rclcpp::Clock().now();

    message.encoding = "rgba8";
    message.height = static_cast<sensor_msgs::msg::Image::_height_type>(750);
    message.width = static_cast<sensor_msgs::msg::Image::_width_type>(750);
    message.step = static_cast<sensor_msgs::msg::Image::_step_type>(750 * 4);
    message.data = random_image_data_;

    publisher->publish(message);
  }

  std::vector<uint8_t> random_image_data_;
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher;
};

}  // namespace nodes
