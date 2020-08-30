// description

#ifndef ROSBAG2_EXTENSIONS__DATASTREAMBASE_HPP_
#define ROSBAG2_EXTENSIONS__DATASTREAMBASE_HPP_
#include <iostream>
#include <cstdio>
#include <memory>
#include <string>

#include "rcutils/time.h"
#include "rcpputils/filesystem_helper.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rclcpp/serialization.hpp"


#include "rosbag2_cpp/readers/random_access_reader.hpp"
#include "rosbag2_cpp/writers/sequential_writer.hpp"

#include "test_msgs/msg/basic_types.hpp"

using TestMsgT = test_msgs::msg::BasicTypes;

namespace rosbag2_extensions
{

class DataStreamBase
{
public:
    DataStreamBase(const std::string &data_directory,const std::string &stream_name);
    ~DataStreamBase();

protected:
    
    rosbag2_cpp::StorageOptions storage_options_;
    rosbag2_cpp::ConverterOptions converter_options_;

    
    rcpputils::fs::path data_directory_;
    std::string stream_name_;   
    bool opened_;
    rclcpp::Serialization<TestMsgT> serialization_;

};


}
#endif // ROSBAG2_EXTENSIONS__DATASTREAMBASE_HPP_