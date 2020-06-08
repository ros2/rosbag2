#include "rclcpp/rclcpp.hpp"
#include "pointcloud2_publisher.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nodes::PointCloud2Publisher>("pointcloud2_publisher", "pointcloud2");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
