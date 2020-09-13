#include <iostream>
#include <cstdio>
#include <memory>
#include <string>
#include <chrono>
#include <thread>
#include <utility>
#include <any>

#include "rcpputils/filesystem_helper.hpp"
#include "rcutils/time.h"

#include "vtr_storage/DataStreamReader.hpp"
#include "vtr_storage/DataStreamWriter.hpp"

#include "test_msgs/msg/basic_types.hpp"

// sample code showing how to use the data streams
int main()
{
  using TestMsgT = test_msgs::msg::BasicTypes;
  TestMsgT test_msg;

  // create
  vtr::storage::DataStreamWriter<TestMsgT> writer("/home/daniel/test/ROS2BagFileParsing/dev_ws/test_rosbag2_writer_api_bag", "test_stream");
  writer.open();
  for (int i = 1; i <= 10; i++) {
    test_msg.float64_value = i*10;
    int32_t index_return = writer.write(std::any(test_msg));
    std::cout << index_return << std::endl;
  }
  writer.close();

  // append
  vtr::storage::DataStreamWriter<TestMsgT> writer2("/home/daniel/test/ROS2BagFileParsing/dev_ws/test_rosbag2_writer_api_bag", "test_stream", true);
  writer2.open();
  for (int i = 11; i <= 20; i++) {
    test_msg.float64_value = i*10;
    int32_t index_return = writer2.write(std::any(test_msg));
    std::cout << index_return << std::endl;
  }
  // writer2.close();

  // read
  vtr::storage::DataStreamReader<TestMsgT> reader("/home/daniel/test/ROS2BagFileParsing/dev_ws/test_rosbag2_writer_api_bag", "test_stream");
  test_msg.float64_value = std::any_cast<TestMsgT>(*(reader.readAtIndex(5))).float64_value;
  std::cout << test_msg.float64_value << std::endl;

  std::cout << "~~~~~~~~~~~~~~~~~~~~" << std::endl;
  auto bag_message_vector = reader.readAtIndexRange(1, 9);
  for (auto message : *bag_message_vector) {
    std::cout << std::any_cast<TestMsgT>(*message).float64_value << std::endl;
  }

  writer2.close();
  // append and read
  writer2.open();
  for (int i = 21; i <= 30; i++) {
    test_msg.float64_value = i*10;
    int32_t index_return = writer2.write(std::any(test_msg));
    test_msg.float64_value = std::any_cast<TestMsgT>(*(reader.readAtIndex(i))).float64_value;
    std::cout << "Written index: " << index_return << ", Read: " << test_msg.float64_value << std::endl;
  }
  // writer auto closes when it goes out of scope
  // writer2.close(); // when writer closes, it writes the metadata.yaml
}