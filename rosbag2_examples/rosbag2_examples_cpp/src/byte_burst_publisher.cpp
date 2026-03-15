#include <chrono>
#include <cstdint>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"

using namespace std::chrono_literals;

namespace
{

std::vector<uint8_t> random_bytes(size_t size)
{
  std::vector<uint8_t> data(size);
  std::mt19937 gen(12345);
  std::uniform_int_distribution<int> dist(0, 255);
  for (auto & value : data) {
    value = static_cast<uint8_t>(dist(gen));
  }
  return data;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("byte_burst_publisher");

  const auto mode = node->declare_parameter<std::string>("mode", "small");
  const auto small_count = node->declare_parameter<int>("small_count", 1000000);
  const auto big_mib = node->declare_parameter<int>("big_mib", 1);
  const auto big_count = node->declare_parameter<int>("big_count", 512);
  const auto create_many_topics = node->declare_parameter<bool>("create_many_topics", true);

  if (mode == "small") {
    auto pub = node->create_publisher<std_msgs::msg::ByteMultiArray>("/small_topic", 1);
    rclcpp::sleep_for(500ms);
    auto payload = random_bytes(64);
    std_msgs::msg::ByteMultiArray msg;
    msg.data = payload;
    for (int i = 0; i < small_count; ++i) {
      pub->publish(msg);
    }
    RCLCPP_INFO(node->get_logger(), "published %ld small messages", small_count);
  } else if (mode == "big") {
    std::vector<rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr> pubs;
    if (create_many_topics) {
      pubs.reserve(100);
      for (int i = 0; i < 100; ++i) {
        pubs.push_back(
          node->create_publisher<std_msgs::msg::ByteMultiArray>("/many/t" + std::to_string(i), 1));
      }
    }
    auto big_pub = node->create_publisher<std_msgs::msg::ByteMultiArray>("/big_topic", 10);
    rclcpp::sleep_for(500ms);

    if (create_many_topics) {
      auto tiny_payload = random_bytes(16);
      std_msgs::msg::ByteMultiArray tiny_msg;
      tiny_msg.data = tiny_payload;
      for (const auto & pub : pubs) {
        pub->publish(tiny_msg);
      }
    }

    auto payload = random_bytes(static_cast<size_t>(big_mib) * 1024u * 1024u);
    std_msgs::msg::ByteMultiArray msg;
    msg.data = payload;
    for (int i = 0; i < big_count; ++i) {
      big_pub->publish(msg);
    }
    RCLCPP_INFO(
      node->get_logger(), "published %ld big messages of %ld MiB", big_count, big_mib);
  } else {
    RCLCPP_ERROR(node->get_logger(), "unknown mode: %s", mode.c_str());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::sleep_for(1s);
  rclcpp::shutdown();
  return 0;
}
