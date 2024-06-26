// Copyright 2024 Sony Group Corporation.
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

#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>

#include "example_interfaces/msg/string.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_transport/reader_writer_factory.hpp"

using namespace std::chrono_literals;

class SimpleBagReader : public rclcpp::Node
{
public:
  explicit SimpleBagReader(const std::string & bag_filename)
  : Node("simple_bag_reader")
  {
    publisher_ = this->create_generic_publisher(
      "chatter", "example_interfaces/msg/String", 10);

    timer_ = this->create_wall_timer(
      100ms,
      [this]() {return this->timer_callback();}
    );

    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_filename;
    reader_ = rosbag2_transport::ReaderWriterFactory::make_reader(storage_options);
    reader_->open(storage_options);
  }

private:
  void timer_callback()
  {
    while (reader_->has_next()) {
      rosbag2_storage::SerializedBagMessageSharedPtr msg = reader_->read_next();

      if (msg->topic_name != "chatter") {
        continue;
      }
      rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
      std::cout << "Publish serialized data to " << msg->topic_name << ".\n";
      publisher_->publish(serialized_msg);
      break;
    }
  }
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<rclcpp::GenericPublisher> publisher_;

  std::unique_ptr<rosbag2_cpp::Reader> reader_;
};

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <bag>" << std::endl;
    return 1;
  }
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleBagReader>(argv[1]));
  rclcpp::shutdown();
  return 0;
}
