#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class PlaybackNode : public rclcpp::Node
{
  public:
    PlaybackNode(const std::string & bag_filename)
    : Node("playback_node")
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);

      timer_ = this->create_wall_timer(100ms,
          [this](){return this->timer_callback();}
      );

      reader_.open(bag_filename);
    }

  private:
    void timer_callback()
    {
      while (reader_.has_next()) {
        rosbag2_storage::SerializedBagMessageSharedPtr msg = reader_.read_next();

        if (msg->topic_name != "/turtle1/pose") {
          continue;
        }

        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
        std_msgs::msg::String::SharedPtr ros_msg = std::make_shared<std_msgs::msg::String>();

        serialization_.deserialize_message(&serialized_msg, ros_msg.get());

        publisher_->publish(*ros_msg);
        std::cout << ros_msg->data << "\n";

        break;
      }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

    rclcpp::Serialization<std_msgs::msg::String> serialization_;
    rosbag2_cpp::Reader reader_;
};

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <bag>" << std::endl;
    return 1;
  }

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlaybackNode>(argv[1]));
  rclcpp::shutdown();

  return 0;
}