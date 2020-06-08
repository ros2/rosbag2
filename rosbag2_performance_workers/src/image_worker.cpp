#include "rclcpp/rclcpp.hpp"
#include "image_publisher.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nodes::ImagePublisher>("image_publisher", "image");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
