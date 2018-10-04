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

#include "formatter.hpp"
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

void Rosbag2Transport::print_bag_info(const std::string & uri)
{
  auto metadata = factory_->create_info()->read_metadata(uri);
  auto start_time = metadata.starting_time.time_since_epoch();
  auto end_time = start_time + metadata.duration;
  auto formatter = std::make_unique<Formatter>();
  std::stringstream info_stream;
  int indentation_spaces = 18;  // just the longest info field (Topics with Type:) plus one space.

  info_stream << std::endl;
  info_stream << "Files:            ";
  formatter->format_file_paths(metadata.relative_file_paths, info_stream, indentation_spaces);
  info_stream << "Bag size:         " << formatter->format_file_size(
    metadata.bag_size) << std::endl;
  info_stream << "Storage id:       " << metadata.storage_identifier << std::endl;
  info_stream << "Storage format:   " << metadata.storage_format << std::endl;
  info_stream << "Duration:         " << formatter->format_duration(
    metadata.duration)["time_in_sec"] << "s" << std::endl;
  info_stream << "Start:            " << formatter->format_time_point(start_time) << std::endl;
  info_stream << "End               " << formatter->format_time_point(end_time) << std::endl;
  info_stream << "Messages:         " << metadata.message_count << std::endl;
  info_stream << "Topics with Type: ";
  formatter->format_topics_with_type(
    metadata.topics_with_message_count, info_stream, indentation_spaces);

  std::cout << info_stream.str() << std::endl;
}

}  // namespace rosbag2_transport
