#include "rosbag2_transport/player.hpp"

namespace rosbag2 {
class Player : public rosbag2_transport::Player {
    using rosbag2_transport::Player::Player;
};
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be
// discoverable when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rosbag2::Player)