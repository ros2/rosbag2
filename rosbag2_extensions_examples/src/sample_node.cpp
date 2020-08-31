#include <iostream>
#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"

#include "rcpputils/filesystem_helper.hpp"
#include "rcutils/time.h"

#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_cpp/readers/random_access_reader.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_cpp/writers/sequential_writer.hpp"

#include "test_msgs/msg/basic_types.hpp"

int main()
{
  std::cout << "Use sample_node_2 instead" << std::endl;
  return 0;
  
  using TestMsgT = test_msgs::msg::BasicTypes;
  TestMsgT test_msg;
  rclcpp::Serialization<TestMsgT> serialization;
  rclcpp::SerializedMessage serialized_msg;

  // test_msg.float64_value = 12345.6789;
  // serialization.serialize_message(&test_msg, &serialized_msg);

  // TestMsgT test_msg2;
  // test_msg2.float64_value = 98765.4321;
  // rclcpp::SerializedMessage serialized_msg2;

  // serialization.serialize_message(&test_msg2, &serialized_msg2);

  auto rosbag_directory = rcpputils::fs::path("test_rosbag2_writer_api_bag");

  // in case the bag was previously not cleaned up
  rcpputils::fs::remove_all(rosbag_directory);
  // See https://github.com/ros2/rosbag2/issues/448
  rcpputils::fs::create_directories(rosbag_directory);

  rosbag2_cpp::StorageOptions storage_options;
  storage_options.uri = rosbag_directory.string();
  storage_options.storage_id = "sqlite3";
  storage_options.max_bagfile_size = 0;  // default
  storage_options.max_cache_size = 0;  // default
  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";

  // writing data to bag
  std::cout << "Writing data to bag" << std::endl;
  {
    rosbag2_cpp::Writer writer(std::make_unique<rosbag2_cpp::writers::SequentialWriter>());
    writer.open(storage_options, converter_options);

    rosbag2_storage::TopicMetadata tm;
    tm.name = "/my/test/topic";
    tm.type = "test_msgs/msg/BasicTypes";
    tm.serialization_format = "cdr";
    writer.create_topic(tm);

    for (int i=1; i<=10; i++) {
      auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
      auto ret = rcutils_system_time_now(&bag_message->time_stamp);
      if (ret != RCL_RET_OK) {
        throw std::runtime_error("couldn't assign time rosbag message");
      }
      bag_message->topic_name = tm.name;
      test_msg.float64_value = i*10;
      serialization.serialize_message(&test_msg, &serialized_msg);

      bag_message->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
        &serialized_msg.get_rcl_serialized_message(), [](rcutils_uint8_array_t * /* data */) {});

      writer.write(bag_message);
    }

    // writer close on scope exit
  }
  std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
  std::cout << "Reading the bag sequentially (starting from index 1)" << std::endl;
  // Read the bag sequentially
  {
    rosbag2_cpp::Reader reader(std::make_unique<rosbag2_cpp::readers::SequentialReader>());
    reader.open(storage_options, converter_options);
    while (reader.has_next()) {
      auto bag_message = reader.read_next();

      TestMsgT extracted_test_msg;
      rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
      serialization.deserialize_message(
        &extracted_serialized_msg, &extracted_test_msg);

      std::cout << extracted_test_msg.float64_value << std::endl;
    }
    // reader closes on scope exit
  }

  std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
  std::cout << "Randomly accessing messages in the bag at indices: {3, 6, 2, 1, 10}" << std::endl;

  // Randomly access a few indices
  {
    rosbag2_cpp::readers::RandomAccessReader reader;
    reader.open(storage_options, converter_options);
    int indices [] = {3, 6, 2, 1, 10};
    for (int i : indices) {
      auto bag_message = reader.read_at_index(i);

      TestMsgT extracted_test_msg;
      rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
      serialization.deserialize_message(
        &extracted_serialized_msg, &extracted_test_msg);

      std::cout << extracted_test_msg.float64_value << std::endl;
    }
    // reader closes on scope exit
  }


  std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
  std::cout << "Randomly accessing messages between indices 3 and 10" << std::endl;

  // Randomly access a few indices
  {
    rosbag2_cpp::readers::RandomAccessReader reader;
    reader.open(storage_options, converter_options);
    int index_begin = 3;
    int index_end = 10;
    auto bag_message_vector = reader.read_at_index_range(index_begin, index_end);
    std::cout << "Number of messages: " << bag_message_vector->size() << std::endl;
    for (auto bag_message : *bag_message_vector) {
      TestMsgT extracted_test_msg;
      rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
      serialization.deserialize_message(
        &extracted_serialized_msg, &extracted_test_msg);

      std::cout << extracted_test_msg.float64_value << std::endl;
    }
    // reader closes on scope exit
  }

  // remove the rosbag again after the test
  rcpputils::fs::remove_all(rosbag_directory);
}