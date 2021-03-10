// Copyright 2018-2021, Bosch Software Innovations GmbH.
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

#ifndef ROSBAG2_PY__FORMAT_BAG_METADATA_HPP_
#define ROSBAG2_PY__FORMAT_BAG_METADATA_HPP_

#include <chrono>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#ifdef _WIN32
#include <time.h>
#endif

#include "rosbag2_storage/bag_metadata.hpp"

namespace details
{

void indent(std::stringstream & info_stream, int number_of_spaces)
{
  info_stream << std::string(number_of_spaces, ' ');
}

std::unordered_map<std::string, std::string> format_duration(
  std::chrono::high_resolution_clock::duration duration)
{
  std::unordered_map<std::string, std::string> formatted_duration;
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

std::string format_time_point(
  std::chrono::high_resolution_clock::duration duration)
{
  auto formatted_duration = format_duration(duration);
  return formatted_duration["date"] + " " + formatted_duration["time"] +
         " (" + formatted_duration["time_in_sec"] + ")";
}

std::string format_file_size(uint64_t file_size)
{
  double size = static_cast<double>(file_size);
  static const char * units[] = {"B", "KiB", "MiB", "GiB", "TiB"};
  double reference_number_bytes = 1024;
  int index = 0;
  while (size >= reference_number_bytes && index < 4) {
    size /= reference_number_bytes;
    index++;
  }

  std::stringstream rounded_size;
  int size_format_precision = index == 0 ? 0 : 1;
  rounded_size << std::setprecision(size_format_precision) << std::fixed << size;
  return rounded_size.str() + " " + units[index];
}

void format_file_paths(
  const std::vector<std::string> & paths,
  std::stringstream & info_stream,
  int indentation_spaces)
{
  if (paths.empty()) {
    info_stream << std::endl;
    return;
  }

  info_stream << paths[0] << std::endl;
  size_t number_of_files = paths.size();
  for (size_t i = 1; i < number_of_files; i++) {
    indent(info_stream, indentation_spaces);
    info_stream << paths[i] << std::endl;
  }
}

void format_topics_with_type(
  const std::vector<rosbag2_storage::TopicInformation> & topics,
  std::stringstream & info_stream,
  int indentation_spaces)
{
  if (topics.empty()) {
    info_stream << std::endl;
    return;
  }

  auto print_topic_info =
    [&info_stream](const rosbag2_storage::TopicInformation & ti) -> void {
      info_stream << "Topic: " << ti.topic_metadata.name << " | ";
      info_stream << "Type: " << ti.topic_metadata.type << " | ";
      info_stream << "Count: " << ti.message_count << " | ";
      info_stream << "Serialization Format: " << ti.topic_metadata.serialization_format;
      info_stream << std::endl;
    };

  print_topic_info(topics[0]);
  size_t number_of_topics = topics.size();
  for (size_t j = 1; j < number_of_topics; ++j) {
    indent(info_stream, indentation_spaces);
    print_topic_info(topics[j]);
  }
}

}  // namespace details

inline std::string format_bag_meta_data(const rosbag2_storage::BagMetadata & metadata)
{
  auto start_time = metadata.starting_time.time_since_epoch();
  auto end_time = start_time + metadata.duration;
  std::stringstream info_stream;
  int indentation_spaces = 19;  // The longest info field (Topics with Type:) plus one space.

  info_stream << std::endl;
  info_stream << "Files:             ";
  details::format_file_paths(metadata.relative_file_paths, info_stream, indentation_spaces);
  info_stream << "Bag size:          " << details::format_file_size(
    metadata.bag_size) << std::endl;
  info_stream << "Storage id:        " << metadata.storage_identifier << std::endl;
  info_stream << "Duration:          " << details::format_duration(
    metadata.duration)["time_in_sec"] << "s" << std::endl;
  info_stream << "Start:             " << details::format_time_point(start_time) <<
    std::endl;
  info_stream << "End:               " << details::format_time_point(end_time) << std::endl;
  info_stream << "Messages:          " << metadata.message_count << std::endl;
  info_stream << "Topic information: ";
  details::format_topics_with_type(
    metadata.topics_with_message_count, info_stream, indentation_spaces);

  return info_stream.str();
}

#endif  // ROSBAG2_PY__FORMAT_BAG_METADATA_HPP_
