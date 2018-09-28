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

#include "rosbag2/formatter.hpp"

#include <chrono>
#include <iomanip>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#ifdef _WIN32
#include <time.h>
#endif

namespace rosbag2
{

std::map<std::string, std::string> Formatter::format_duration(
  std::chrono::high_resolution_clock::duration duration)
{
  std::map<std::string, std::string> formatted_duration;
  auto m_seconds = std::chrono::duration_cast<std::chrono::milliseconds>(duration);
  auto seconds = std::chrono::duration_cast<std::chrono::seconds>(m_seconds);
  std::string fractional_seconds = std::to_string(m_seconds.count() % 1000);
  std::time_t std_time_point = seconds.count();
  tm time;
#ifdef _WIN32
  localtime_s(&time, &std_time_point);
#else
  localtime_r(&std_time_point, &time);
#endif

  std::stringstream formatted_date;
  std::stringstream formatted_time;
  formatted_date << std::put_time(&time, "%b %e %Y");
  formatted_time << std::put_time(&time, "%H:%M:%S") << "." << fractional_seconds;
  formatted_duration["date"] = formatted_date.str();
  formatted_duration["time"] = formatted_time.str();
  formatted_duration["time_in_sec"] = std::to_string(seconds.count()) + "." + fractional_seconds;

  return formatted_duration;
}

std::string Formatter::format_time_point(
  std::chrono::high_resolution_clock::duration duration)
{
  auto formatted_duration = format_duration(duration);
  return formatted_duration["date"] + " " + formatted_duration["time"] +
         " (" + formatted_duration["time_in_sec"] + ")";
}

std::string Formatter::format_file_size(size_t file_size)
{
  if (file_size == 0) {
    return "0 B";
  }

  double size = static_cast<double>(file_size);
  static const char * units[] = {"B", "KiB", "MiB", "GiB", "TiB"};
  double reference_number_bytes = 1024;
  int index = 0;
  while (size >= reference_number_bytes && index < 4) {
    size /= reference_number_bytes;
    index++;
  }

  std::stringstream rounded_size;
  rounded_size << std::setprecision(1) << std::fixed << size;  // round to 1 decimal digits.
  return rounded_size.str() + " " + units[index];
}

void Formatter::format_file_paths(std::vector<std::string> paths, std::stringstream & info_stream)
{
  if (paths.empty()) {
    info_stream << "\n";
    return;
  }

  size_t number_of_files = paths.size();
  for (size_t i = 0; i < number_of_files; i++) {
    if (i == 0) {
      info_stream << paths[i] << "\n";
    } else {
      info_stream << "                  " << paths[i] << "\n";
    }
  }
}

void Formatter::format_topics_with_type(
  std::vector<rosbag2::TopicMetadata> topics, std::stringstream & info_stream)
{
  if (topics.empty()) {
    info_stream << "\n";
    return;
  }

  size_t number_of_topics = topics.size();
  for (size_t j = 0; j < number_of_topics; ++j) {
    std::string topic_with_type = topics[j].topic_with_type.name + "; " +
      topics[j].topic_with_type.type + "; " + std::to_string(topics[j].message_count) + " msgs\n";
    if (j == 0) {
      info_stream << topic_with_type;
    } else {
      info_stream << "                  " << topic_with_type;
    }
  }
}

}  // namespace rosbag2
