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

#ifndef ROSBAG2__FORMAT_CONVERTER_FACTORY_HPP_
#define ROSBAG2__FORMAT_CONVERTER_FACTORY_HPP_

#include <memory>
#include <string>

#include "rosbag2/format_converter_interface.hpp"
#include "rosbag2/visibility_control.hpp"

namespace rosbag2
{

class ROSBAG2_PUBLIC FormatConverterFactory
{
public:
  virtual ~FormatConverterFactory() = default;

  virtual std::shared_ptr<FormatConverterInterface> load_converter(const std::string & format) = 0;
};

}  // namespace rosbag2

#endif  // ROSBAG2__FORMAT_CONVERTER_FACTORY_HPP_
