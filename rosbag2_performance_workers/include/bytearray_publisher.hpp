#include <chrono>
#include <string>
#include <vector>
#include <memory>

#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

std::vector<uint8_t> randomByteArrayData(size_t size) {
  std::vector<uint8_t> byte(size, 0);

  for (size_t i = 0; i < size; ++i) {
    byte[i] = std::rand() % 255;
  }
  return byte;
}

namespace nodes {

class ByteArrayPublisher : public rclcpp::Node {
public:
  ByteArrayPublisher(const std::string & name, const std::string & topic)
  : Node(name)
  {
    this->declare_parameter("frequency");
    this->declare_parameter("max_count");
    this->declare_parameter("size");
    this->declare_parameter("delay");
    this->declare_parameter("benchmark_path");

    auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this);
    while (!parameters_client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        rclcpp::shutdown();
      }
      RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }

    frequency = parameters_client->get_parameter<uint32_t>("frequency", 100);
    if(frequency == 0) {
      RCLCPP_ERROR(this->get_logger(), "Frequency can't be 0. Exiting.");
      rclcpp::shutdown(nullptr, "frequency error");
      return;
    }

    max_count = parameters_client->get_parameter<uint32_t>("max_count", 100);
    size = parameters_client->get_parameter<uint32_t>("size", 1024);
    delay = parameters_client->get_parameter<uint32_t>("delay", 0);
    benchmark_path = parameters_client->get_parameter<std::string>("benchmark_path", "");

    publisher = this->create_publisher<std_msgs::msg::ByteMultiArray>(topic, 10);
    delay_timer = this->create_wall_timer(std::chrono::milliseconds(delay), std::bind(&ByteArrayPublisher::delay_callback, this));
    random_bytearray_data = randomByteArrayData(size);  // image step * height
  }

private:
  void delay_callback()
  {
    std::cout << this->get_name() <<  ": Delay " << delay << " finished" << std::endl;
    timer = this->create_wall_timer(std::chrono::milliseconds(1000/frequency), std::bind(&ByteArrayPublisher::timer_callback, this));
    delay_timer->cancel();
  }
  void timer_callback()
  {
    uint32_t static current_msg_count = 0;

    auto message = std_msgs::msg::ByteMultiArray();
    message.data = random_bytearray_data;
    publisher->publish(message);

    std::cout << this->get_name() << ": " << current_msg_count << std::endl;
    if(++current_msg_count == max_count) {
      timer->cancel();
      rclcpp::shutdown();
    }

    //(piotr.jaroszek) TODO: raport data here and save to benchmark_path
  }

  std::vector<uint8_t> random_bytearray_data;
  std::string benchmark_path;
  uint32_t frequency;
  uint32_t max_count;
  uint32_t size;
  uint32_t delay;
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::TimerBase::SharedPtr delay_timer;
  rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr publisher;
};

}
