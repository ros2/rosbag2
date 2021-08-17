// Copyright 2021, Open Source Robotics Foundation, Inc.
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


#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <example_interfaces/msg/int32.hpp>

#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>

#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class DataGenerator : public rclcpp::Node
{
public:
  DataGenerator()
  : Node("data_generator")
  {
    data.data = 0;
    const rosbag2_cpp::StorageOptions storage_options({"timed_synthetic_bag", "sqlite3"});
    const rosbag2_cpp::ConverterOptions converter_options(
      {rmw_get_serialization_format(),
        rmw_get_serialization_format()});
    writer_ = std::make_unique<rosbag2_cpp::writers::SequentialWriter>();

    writer_->open(storage_options, converter_options);

    writer_->create_topic(
      {"synthetic",
        "example_interfaces/msg/Int32",
        rmw_get_serialization_format(),
        ""});

    timer_ = create_wall_timer(1s, std::bind(&DataGenerator::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto serializer = rclcpp::Serialization<example_interfaces::msg::Int32>();
    auto serialized_message = rclcpp::SerializedMessage();
    serializer.serialize_message(&data, &serialized_message);

    auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();

    bag_message->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
      new rcutils_uint8_array_t,
      [this](rcutils_uint8_array_t * msg) {
        auto fini_return = rcutils_uint8_array_fini(msg);
        delete msg;
        if (fini_return != RCUTILS_RET_OK) {
          RCLCPP_ERROR(
            get_logger(),
            "Failed to destroy serialized message %s",
            rcutils_get_error_string().str);
        }
      });
    *bag_message->serialized_data = serialized_message.release_rcl_serialized_message();

    bag_message->topic_name = "synthetic";
    if (rcutils_system_time_now(&bag_message->time_stamp) != RCUTILS_RET_OK) {
      RCLCPP_ERROR(
        get_logger(),
        "Error getting current time: %s",
        rcutils_get_error_string().str);
    }

    writer_->write(bag_message);
    ++data.data;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<rosbag2_cpp::writers::SequentialWriter> writer_;
  example_interfaces::msg::Int32 data;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DataGenerator>());
  rclcpp::shutdown();
  return 0;
}
