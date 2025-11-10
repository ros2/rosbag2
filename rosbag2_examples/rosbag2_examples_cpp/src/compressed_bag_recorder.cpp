// Copyright 2025 Open Source Robotics Foundation
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

#include "example_interfaces/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rosbag2_compression/compression_options.hpp"
#include "rosbag2_compression/sequential_compression_writer.hpp"

#include "rosbag2_cpp/writer.hpp"

using std::placeholders::_1;

class CompressedBagRecorder : public rclcpp::Node {
public:
  CompressedBagRecorder()
  : Node("compressed_bag_recorder")
  {
    rosbag2_compression::CompressionOptions compression_options;

    compression_options.compression_format = "zstd";
    compression_options.compression_mode = rosbag2_compression::CompressionMode::FILE;

    auto compressed_writer = std::make_unique<rosbag2_compression::SequentialCompressionWriter>(
      compression_options);

    writer_ = std::make_unique<rosbag2_cpp::Writer>(std::move(compressed_writer));
    writer_->open("my_bag");

    subscription_ = this->create_subscription<example_interfaces::msg::String>(
      "chatter", 10, std::bind(&CompressedBagRecorder::topic_callback, this, _1));
  }

private:
  void topic_callback(std::shared_ptr<const rclcpp::SerializedMessage> msg) const
  {
    rclcpp::Time time_stamp = this->now();

    writer_->write(msg, "chatter", "example_interfaces/msg/String", time_stamp);
  }

  rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr subscription_;
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CompressedBagRecorder>());
  rclcpp::shutdown();
  return 0;
}
