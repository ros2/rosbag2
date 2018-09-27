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

#include "rosbag2/info.hpp"

#include <chrono>
#include <iomanip>
#include <map>
#include <memory>
#include <string>
#include <sstream>
#include <utility>

#include "rosbag2_storage/metadata_io.hpp"

namespace rosbag2
{

Info::Info(std::shared_ptr<rosbag2_storage::Rosbag2StorageFactory> storage_factory)
: storage_factory_(std::move(storage_factory))
{}

rosbag2::BagMetadata Info::read_metadata(const std::string & uri)
{
  return storage_factory_->metadata_io()->read_metadata(uri);
}

std::map<std::string, std::string> Info::format_duration(
  std::chrono::high_resolution_clock::duration time_point)
{
  std::map<std::string, std::string> formatted_duration;
  auto m_seconds =
    std::chrono::duration_cast<std::chrono::milliseconds>(time_point);
  auto seconds = std::chrono::duration_cast<std::chrono::seconds>(m_seconds);
  std::time_t std_time_point = seconds.count();
  tm time;
  localtime_r(&std_time_point, &time);
  std::string fractional_seconds = std::to_string(m_seconds.count() % 1000);

  char formatted_time_char[50];
  strftime(formatted_time_char, sizeof(formatted_time_char) - 1, "%b %d %Y", &time);
  formatted_duration["date"] = std::string(formatted_time_char);
  strftime(formatted_time_char, sizeof(formatted_time_char) - 1, "%H:%M:%S", &time);
  formatted_duration["time"] = std::string(formatted_time_char);
  formatted_duration["time_in_sec"] = std::to_string(seconds.count()) + "." + fractional_seconds;
  formatted_duration["fractional_seconds"] = fractional_seconds;

  return formatted_duration;
}

std::string Info::format_time_point(
  std::chrono::high_resolution_clock::duration duration)
{
  auto formatted_duration = format_duration(duration);
  return formatted_duration["date"] + " " + formatted_duration["time"] + "." +
         formatted_duration["fractional_seconds"] + " (" + formatted_duration["time_in_sec"] + ")";
}

std::string Info::format_file_size(double file_size)
{
  static const char * units[] = {"B", "KB", "MB", "GB", "TB"};
  double reference_number_bytes = 1024;
  int index = 0;
  while (file_size > reference_number_bytes && index < 5) {
    file_size /= reference_number_bytes;
    index++;
  }

  std::stringstream rounded_size;
  rounded_size << std::setprecision(2) << std::fixed << file_size;  // round to 2 decimal digits.
  return rounded_size.str() + " " + units[index];
}

}  // namespace rosbag2
