#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_storage/qos.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "example_interfaces/msg/string.hpp"

using namespace std::chrono_literals;

class PlaybackNode : public rclcpp::Node
{
  public:
    PlaybackNode(const std::string & bag_filename)
    : Node("playback_node")
    {
      reader_.open(bag_filename);

      auto topics = reader_.get_all_topics_and_types();

      for (const auto & topic : topics) {
        if (topic.name == TOPIC_NAME) {
          publisher_ = create_generic_publisher(topic.name, topic.type, rosbag2_storage::Rosbag2QoS{});
        }
      }

      timer_ = this->create_wall_timer(100ms,
          [this](){return this->timer_callback();}
      );
    }

  private:
    void timer_callback()
    {
      while (reader_.has_next()) {
        rosbag2_storage::SerializedBagMessageSharedPtr msg = reader_.read_next();
        if (msg->topic_name != TOPIC_NAME) {
          continue;
        }
        publisher_->publish(rclcpp::SerializedMessage(*msg->serialized_data));

        break;
      }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<rclcpp::GenericPublisher> publisher_;

    rosbag2_cpp::Reader reader_;

    const std::string TOPIC_NAME {"chatter"};
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