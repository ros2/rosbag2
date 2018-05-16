/*
 *  Copyright (c) 2018,  Bosch Software Innovations GmbH.
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

#ifndef ROS2_ROSBAG_EVALUATION_PROFILER_H
#define ROS2_ROSBAG_EVALUATION_PROFILER_H

#include <chrono>
#include <vector>
#include <functional>

#include "benchmark/benchmark.h"

namespace ros2bag
{

class Profiler
{
public:
  Profiler(
    std::vector<std::pair<std::string, std::string>> const & meta_data,
    std::string const & file_name)
    : meta_data_(meta_data), file_name_(file_name)
  {}

  ~Profiler() = default;

  void take_time_for(std::string const & task);

  void track_disk_usage();

  std::string csv_header() const;

  std::string csv_entry() const;

  using TickProgress = std::function<void()>;

  TickProgress measure_progress(
    std::string const & subject,
    unsigned long const total,
    unsigned long const increment = 1
  );

private:
  std::string file_name_;
  long disk_usage_;
  std::vector<std::pair<std::string, std::string>> meta_data_;
  std::vector<std::pair<std::string, std::chrono::system_clock::time_point>> time_points_;
};

}

#endif //ROS2_ROSBAG_EVALUATION_PROFILER_H
