#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"

using namespace std::chrono_literals;

class ByteSubscriberProbe : public rclcpp::Node
{
public:
  ByteSubscriberProbe()
  : Node("byte_subscriber_probe")
  {
    const auto topic = this->declare_parameter<std::string>("topic", "/big_topic");
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    sub_ = this->create_subscription<std_msgs::msg::ByteMultiArray>(
      topic, qos,
      [this](std_msgs::msg::ByteMultiArray::ConstSharedPtr msg) {
        message_count_.fetch_add(1, std::memory_order_relaxed);
        const auto size = msg->data.size();
        auto current = max_payload_bytes_.load(std::memory_order_relaxed);
        while (
          size > current &&
          !max_payload_bytes_.compare_exchange_weak(current, size, std::memory_order_relaxed))
        {
        }
      });

    timer_ = this->create_wall_timer(5s, [this]() {
      RCLCPP_INFO(
        this->get_logger(), "messages=%zu max_payload_bytes=%zu",
        message_count_.load(std::memory_order_relaxed),
        max_payload_bytes_.load(std::memory_order_relaxed));
    });
  }

private:
  std::atomic<size_t> message_count_{0};
  std::atomic<size_t> max_payload_bytes_{0};
  rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ByteSubscriberProbe>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
