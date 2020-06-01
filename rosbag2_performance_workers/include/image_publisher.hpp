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
#include <memory>

#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/empty.hpp"

using std::placeholders::_1;
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
  ImagePublisher(const std::string & node_name, 
    const std::string & topic_name)
  : Node(node_name)
  {
    this->declare_parameter("dt");
    this->declare_parameter("max_count");
    this->declare_parameter("dimensions");

    auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this);
    while (!parameters_client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        rclcpp::shutdown();
      }
      RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }

    dt = parameters_client->get_parameter<uint32_t>("dt", 10);
    max_count = parameters_client->get_parameter<uint32_t>("max_count", 100);
    dimensions = parameters_client->get_parameter<uint32_t>("dimensions", 1024);

    publisher = this->create_publisher<sensor_msgs::msg::Image>(topic_name, 10);
    done_publisher = this->create_publisher<std_msgs::msg::Empty>("/done", 10);
    // Temporary disabled!
    // subscriber = this->create_subscription<std_msgs::msg::Empty>(
    //   "/ready",
    //   10,
    //   std::bind(&ImagePublisher::run, this, _1));
    
    timer = this->create_wall_timer(std::chrono::milliseconds(dt), std::bind(&ImagePublisher::timer_callback, this));
    random_image_data_ = randomImageData(dimensions * 4 * dimensions);  // image step * height
  }

private:
  // Temporary disabled!
  // void run(const std_msgs::msg::Empty::SharedPtr msg) {
  //   std::cout << "Is running yes" << std::endl;
  //       if (is_running) {
  //         std::cout << "Is running yes" << std::endl;
  //         return;
  //       } else {
  //         std::cout << "Creating wall timer " << dt << std::endl;
  //         timer = this->create_wall_timer(std::chrono::milliseconds(dt), std::bind(&ImagePublisher::timer_callback, this));
  //         std::cout << "Creating wall timer2" << std::endl;
  //         is_running = true;
  //       }
  // }

  void timer_callback()
  {
    uint32_t static current_msg_count = 0;
    auto message = sensor_msgs::msg::Image();
    message.header = std_msgs::msg::Header();
    message.header.frame_id = "image_frame";
    message.header.stamp = rclcpp::Clock().now();

    message.encoding = "rgba8";
    message.height = static_cast<sensor_msgs::msg::Image::_height_type>(dimensions);
    message.width = static_cast<sensor_msgs::msg::Image::_width_type>(dimensions);
    message.step = static_cast<sensor_msgs::msg::Image::_step_type>(dimensions * 4);
    message.data = random_image_data_;

    publisher->publish(message);

    std::cout << current_msg_count << std::endl;
    if(++current_msg_count == max_count) {
      done_publisher->publish(std_msgs::msg::Empty());
      timer->cancel();
      rclcpp::shutdown();
    }
  }

  std::vector<uint8_t> random_image_data_;
  bool is_running = false;
  uint32_t dt;
  uint32_t max_count;
  uint32_t dimensions;
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr done_publisher;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr subscriber;
};

}  // namespace nodes
