// description

#ifndef VTR_STORAGE__DATASTREAMBASE_HPP_
#define VTR_STORAGE__DATASTREAMBASE_HPP_
#include <iostream>
#include <cstdio>
#include <memory>
#include <string>

#include "rcutils/time.h"
#include "rcpputils/filesystem_helper.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_cpp/storage_options.hpp"
#include "rosbag2_cpp/converter_options.hpp"

#include "test_msgs/msg/basic_types.hpp"

using TestMsgT = test_msgs::msg::BasicTypes;

namespace vtr::storage
{

class DataStreamBase
{
public:
    DataStreamBase(const std::string &data_directory_string,const std::string &stream_name);
    ~DataStreamBase();

protected:
    
    rosbag2_cpp::StorageOptions storage_options_;
    rosbag2_cpp::ConverterOptions converter_options_;

    
    rcpputils::fs::path data_directory_;
    std::string stream_name_;   
    bool opened_;
    rclcpp::Serialization<TestMsgT> serialization_;

};


} // namespace vtr::storage
#endif // VTR_STORAGE__DATASTREAMBASE_HPP_