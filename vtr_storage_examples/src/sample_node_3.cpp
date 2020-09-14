#include <any>
#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <utility>

#include "rcpputils/filesystem_helper.hpp"
#include "rcutils/time.h"

#include "vtr_storage/data_bubble.hpp"
#include "vtr_storage/data_stream_reader.hpp"
#include "vtr_storage/data_stream_writer.hpp"

#include "test_msgs/msg/basic_types.hpp"

// sample code showing how to use the data streams
int main() {
  using TestMsgT = test_msgs::msg::BasicTypes;
  TestMsgT test_msg;

  vtr::storage::DataStreamWriter<TestMsgT> writer(
      "/home/daniel/test/ROS2BagFileParsing/dev_ws/test_rosbag2_writer_api_bag",
      "test_stream");

  writer.open();
  for (int i = 1; i <= 10; i++) {
    test_msg.float64_value = i * 10;
    writer.write(std::any(test_msg));
  }
  writer.close();

  auto reader = std::make_shared<vtr::storage::DataStreamReader<TestMsgT>>(
      "/home/daniel/test/ROS2BagFileParsing/dev_ws/test_rosbag2_writer_api_bag",
      "test_stream");
  vtr::storage::DataBubble<TestMsgT> bubble;
  bubble.initialize(
      std::static_pointer_cast<vtr::storage::DataStreamReaderBase>(reader));
  bubble.setIndices(2, 8);
  bubble.load();
  auto message = bubble.retrieve(3);  // 3 is local index of this bubble, which
                                      // translates to a global index of 2+3=5
  std::cout << std::any_cast<TestMsgT>(message).float64_value << std::endl;

  // test_msg.float64_value = reader.readAtIndex(5)->float64_value;
  // std::cout << test_msg.float64_value << std::endl;

  // std::cout << "~~~~~~~~~~~~~~~~~~~~" << std::endl;
  // auto bag_message_vector = reader.readAtIndexRange(3, 8);
  // for (auto message : *bag_message_vector) {
  //   std::cout << message->float64_value << std::endl;
  // }
}