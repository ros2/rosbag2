// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#ifndef ROSBAG2_TRANSPORT__CONVERTER_HPP_
#define ROSBAG2_TRANSPORT__CONVERTER_HPP_

#include <future>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "rosbag2_cpp/types.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_transport/storage_options.hpp"

namespace rosbag2
{
class Writer;
class Reader;
}

namespace rosbag2_transport
{

class Converter
{
public:
  explicit Converter(
    std::shared_ptr<rosbag2_cpp::Reader> reader,
    std::shared_ptr<rosbag2_cpp::Writer> writer);

  void convert(std::string serialization_format);

private:
  std::shared_ptr<rosbag2_cpp::Reader> reader_;
  std::shared_ptr<rosbag2_cpp::Writer> writer_;
};

}  // namespace rosbag2_transport

#endif  // ROSBAG2_TRANSPORT__CONVERTER_HPP_
