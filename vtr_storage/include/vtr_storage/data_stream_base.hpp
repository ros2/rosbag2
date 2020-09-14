#pragma once

#include <cstdio>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rcpputils/filesystem_helper.hpp"
#include "rcutils/time.h"
#include "rosbag2_cpp/converter_options.hpp"
#include "rosbag2_cpp/storage_options.hpp"

namespace vtr {
namespace storage {

class DataStreamBase {
 public:
  DataStreamBase(const std::string &data_directory_string,
                 const std::string &stream_name);
  ~DataStreamBase();

 protected:
  rosbag2_cpp::StorageOptions storage_options_;
  rosbag2_cpp::ConverterOptions converter_options_;

  rcpputils::fs::path data_directory_;
  std::string stream_name_;
  bool opened_;
};
}  // namespace storage
}  // namespace vtr
