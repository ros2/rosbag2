#include "rosbag2_transport/rosbag2_transport.hpp"

#include "rosbag2/rosbag2.hpp"

namespace rosbag2_transport
{

void Rosbag2Transport::record(const std::vector<std::string> & topic_names)
{
  rclcpp::init(0, nullptr);

  rosbag2::Rosbag2 rosbag2;
  rosbag2.record("test.bag", topic_names);

  rclcpp::shutdown();
}

}  // namespace rosbag2_transport
