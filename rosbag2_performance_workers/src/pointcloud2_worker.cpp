#include "rclcpp/rclcpp.hpp"
#include "pointcloud2_publisher.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nodes::PointCloud2Publisher>("pointcloud2_publisher", "pointcloud2");
  if(rclcpp::ok()) {
    rclcpp::spin(node);
    rclcpp::shutdown();
  }
  return (rclcpp::contexts::get_global_default_context()->shutdown_reason().compare("frequency error") == 0);
}
