// Copyright 2018, Bosch Software Innovations GmbH.
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

#ifndef ROSBAG2_BAG_V2_PLUGINS__STORAGE__ROSBAG_OUTPUT_STREAM_HPP_
#define ROSBAG2_BAG_V2_PLUGINS__STORAGE__ROSBAG_OUTPUT_STREAM_HPP_

#include <cstdint>
#include <memory>
#include <string>

#include "rcutils/types/uint8_array.h"

namespace rosbag2_bag_v2_plugins
{

class RosbagOutputStream
{
public:
  explicit RosbagOutputStream(const std::string & type);

  uint8_t * advance(size_t size);

  std::shared_ptr<rcutils_uint8_array_t> get_content();

private:
  std::shared_ptr<rcutils_uint8_array_t> char_array_;
};

}  // namespace rosbag2_bag_v2_plugins

#endif  // ROSBAG2_BAG_V2_PLUGINS__STORAGE__ROSBAG_OUTPUT_STREAM_HPP_
