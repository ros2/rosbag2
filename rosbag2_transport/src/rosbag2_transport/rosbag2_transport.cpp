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

#include "rosbag2_transport/rosbag2_transport.hpp"

#include <chrono>
#include <ctime>
#include <iomanip>
#include <map>
#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/time.h"

#include "rosbag2/types.hpp"
#include "rosbag2_transport/logging.hpp"

#include "player.hpp"
#include "recorder.hpp"

namespace rosbag2_transport
{

Rosbag2Transport::Rosbag2Transport(std::shared_ptr<rosbag2::Rosbag2Factory> factory)
: factory_(std::move(factory)) {}

void Rosbag2Transport::init()
{
  rclcpp::init(0, nullptr);
}

void Rosbag2Transport::shutdown()
{
  rclcpp::shutdown();
}

void Rosbag2Transport::record(
  const StorageOptions & storage_options, const RecordOptions & record_options)
{
  auto writer = factory_->create_writer(storage_options);
  auto transport_node = setup_node();

  Recorder recorder(writer, transport_node);
  recorder.record(record_options);
}

std::shared_ptr<Rosbag2Node> Rosbag2Transport::setup_node()
{
  if (!transport_node_) {
    transport_node_ = std::make_shared<Rosbag2Node>("rosbag2");
  }
  return transport_node_;
}

void Rosbag2Transport::play(
  const StorageOptions & storage_options, const PlayOptions & play_options)
{
  auto reader = factory_->create_sequential_reader(storage_options);
  auto transport_node = setup_node();

  Player player(reader, transport_node);
  player.play(play_options);
}

std::map<std::string, std::string> Rosbag2Transport::format_duration(
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

std::string Rosbag2Transport::format_time_point(
  std::chrono::high_resolution_clock::duration duration)
{
  auto formatted_duration = format_duration(duration);
  return formatted_duration["date"] + " " + formatted_duration["time"] + "." +
         formatted_duration["fractional_seconds"] + " (" + formatted_duration["time_in_sec"] + ")";
}

std::string Rosbag2Transport::format_file_size(double file_size)
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

void Rosbag2Transport::print_bag_info(const std::string & uri)
{
  auto metadata = factory_->create_info()->read_metadata(uri);
  auto start_time = metadata.starting_time.time_since_epoch();
  auto end_time = start_time + metadata.duration;
  std::stringstream info_stream;

  info_stream << "\nFiles:            " << metadata.relative_file_paths[0] << "\n";
  metadata.relative_file_paths.erase(metadata.relative_file_paths.begin());
  for (const auto & file : metadata.relative_file_paths) {
    info_stream << "                  " << file << "\n";
  }
  info_stream << "Bag size:         " << format_file_size(metadata.bag_size) << "\n";
  info_stream << "Storage id:       " << metadata.storage_identifier << "\n";
  info_stream << "Storage format:   " << metadata.encoding << "\n";
  info_stream << "Duration:         " << format_duration(metadata.duration)["time_in_sec"] << "s\n";
  info_stream << "Start:            " << format_time_point(start_time) << "\n";
  info_stream << "End               " << format_time_point(end_time) << "\n";
  info_stream << "Messages:         " << metadata.message_count << "\n";
  info_stream << "Topics with Type: " <<
    metadata.topics_with_message_count[0].topic_with_type.name <<
    "; " << metadata.topics_with_message_count[0].topic_with_type.type << "; " <<
    std::to_string(metadata.topics_with_message_count[0].message_count) + " msgs\n";
  metadata.topics_with_message_count.erase(metadata.topics_with_message_count.begin());
  for (const auto & topic_with_type_and_count : metadata.topics_with_message_count) {
    info_stream << "                  " << topic_with_type_and_count.topic_with_type.name <<
      "; " << topic_with_type_and_count.topic_with_type.type <<
      "; " << topic_with_type_and_count.message_count << " msgs\n";
  }

  std::cout << info_stream.str() << std::endl;
}

}  // namespace rosbag2_transport
