#include <iostream>
#include <cstdio>
#include <memory>
#include <string>

#include "rcpputils/filesystem_helper.hpp"
#include "rcutils/time.h"

#include "rosbag2_extensions/DataStreamReader.hpp"
#include "rosbag2_extensions/DataStreamWriter.hpp"
#include "rosbag2_extensions/DataBubble.hpp"

#include "test_msgs/msg/basic_types.hpp"

// sample code showing how to use the data streams
int main()
{
  using TestMsgT = test_msgs::msg::BasicTypes;
  TestMsgT test_msg;

  rosbag2_extensions::DataStreamWriter writer("/home/daniel/test/ROS2BagFileParsing/dev_ws/test_rosbag2_writer_api_bag", "test_stream");
  
  writer.open();
  for (int i = 1; i <= 10; i++) {
    test_msg.float64_value = i*10;
    writer.write(test_msg);
  }
  writer.close();

  auto reader = std::make_shared<rosbag2_extensions::DataStreamReader>("/home/daniel/test/ROS2BagFileParsing/dev_ws/test_rosbag2_writer_api_bag", "test_stream");
  rosbag2_extensions::DataBubble bubble;
  bubble.initialize(reader);
  bubble.setIndices(2, 8);
  bubble.load();
  auto message = bubble.retrieve(3); // 3 is local index of this bubble, which translates to a global index of 2+3=5
  std::cout << message.float64_value << std::endl;

  // test_msg.float64_value = reader.readAtIndex(5)->float64_value;
  // std::cout << test_msg.float64_value << std::endl;

  // std::cout << "~~~~~~~~~~~~~~~~~~~~" << std::endl;
  // auto bag_message_vector = reader.readAtIndexRange(3, 8);
  // for (auto message : *bag_message_vector) {
  //   std::cout << message->float64_value << std::endl;
  // }

}