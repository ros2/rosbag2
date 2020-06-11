#include "rclcpp/rclcpp.hpp"
#include "bytearray_publisher.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::Context ctx;
  auto node = std::make_shared<nodes::ByteArrayPublisher>("bytearray_publisher", "bytearray");
  if(rclcpp::ok()) {
    rclcpp::spin(node);
    rclcpp::shutdown();
  }
  return (rclcpp::contexts::get_global_default_context()->shutdown_reason().compare("frequency error") == 0);
}
