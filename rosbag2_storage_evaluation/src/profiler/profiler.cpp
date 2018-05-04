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

#include "profiler/profiler.h"

#include <fstream>
#include <sstream>

using namespace ros2bag;
using namespace std::literals::chrono_literals;

void Profiler::take_time_for(std::string const & task)
{
  time_points_.emplace_back(task, std::chrono::system_clock::now());
}

void Profiler::track_disk_usage()
{
  auto const mode = std::ifstream::binary | std::ifstream::ate;
  std::ifstream file(file_name_, mode);
  disk_usage_ = file.tellg();
}

std::string Profiler::csv_header() const
{
  std::ostringstream header;

  for (auto const & m : meta_data_) {
    header << m.first << ",";
  }

  for (auto const & t : time_points_) {
    header << t.first << " (ms),";
  }

  header << "disk usage (bytes)";

  return header.str();
}

std::string Profiler::csv_entry() const
{
  std::ostringstream entry;

  for (auto const & m : meta_data_) {
    entry << m.second << ",";
  }

  if (!time_points_.empty()) {
    auto const start = time_points_.front().second;
    for (auto const & t : time_points_) {
      auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(t.second - start);
      entry << timestamp.count() << ",";
    }
  }

  entry << disk_usage_;

  return entry.str();
}

Profiler::TickProgress Profiler::measure_progress(
  const std::string & subject,
  unsigned long const total,
  const unsigned long increment)
{
  take_time_for(subject + "_0"/*%*/);

  return [this, next_level = 10, current = 0, subject, total, increment]() mutable {
    current += increment;
    const auto progress = static_cast<double>(current) / total * 100;
    if (progress >= next_level) {
      this->take_time_for(subject + "_" + std::to_string(next_level));
      next_level += 10 /* % */;
    }
  };
}
