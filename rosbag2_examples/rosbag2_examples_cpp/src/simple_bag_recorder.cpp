// Copyright 2021 Open Source Robotics Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>

#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rosbag2_cpp/writer.hpp"

using std::placeholders::_1;

class SimpleBagRecorder : public rclcpp::Node
{
public:
  SimpleBagRecorder()
  : Node("simple_bag_recorder")
  {
    writer_ = std::make_unique<rosbag2_cpp::Writer>();

    writer_->open("my_bag");

    if (message_needs_to_be_edit_before_write_) {
      subscription_ = create_subscription<std_msgs::msg::String>(
        "chatter", 10, std::bind(&SimpleBagRecorder::topic_edit_callback, this, _1));
    } else {
      subscription_ = create_subscription<std_msgs::msg::String>(
        "chatter", 10, std::bind(&SimpleBagRecorder::topic_callback, this, _1));
    }
  }

private:
  void topic_callback(std::shared_ptr<rclcpp::SerializedMessage> msg) const
  {
    rclcpp::Time time_stamp = this->now();
    writer_->write(msg, "chatter", "std_msgs/msg/String", time_stamp);
  }

  void topic_edit_callback(std_msgs::msg::String::UniquePtr msg) const
  {
    rclcpp::Time time_stamp = this->now();
    msg->data += "[edited]";
    auto serialized_msg = std::make_shared<rclcpp::SerializedMessage>();
    serialization_.serialize_message(msg.get(), serialized_msg.get());
    writer_->write(serialized_msg, "chatter", "std_msgs/msg/String", time_stamp);
  }


  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Serialization<std_msgs::msg::String> serialization_;
  std::unique_ptr<rosbag2_cpp::Writer> writer_;

  bool message_needs_to_be_edit_before_write_ {true};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleBagRecorder>());
  rclcpp::shutdown();
  return 0;
}
