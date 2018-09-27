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

void Rosbag2Transport::print_bag_info(const std::string & uri)
{
  auto metadata = factory_->create_info()->read_metadata(uri);
  std::stringstream ss;
  ss << "Storage identifier:  " << metadata.storage_identifier << "\n";
  ss << "File encoding:       " << metadata.encoding << "\n";
  ss << "Associated files (relative paths):\n";
  for (const auto & file : metadata.relative_file_paths) {
    ss << "        - " << file << "\n";
  }
  ss << "Starting time:       " << metadata.starting_time.time_since_epoch().count() << "\n";
  ss << "End time:            " <<
  (metadata.starting_time.time_since_epoch() + metadata.duration).count() << "\n";
  ss << "Duration:            " << metadata.duration.count() << "\n";
  ss << "Total message count: " << metadata.message_count << "\n";
  ss << "Topics with Type and message count:\n";
  for (const auto & topic_with_type_and_count : metadata.topics_with_message_count) {
    ss << "        - " << topic_with_type_and_count.topic_with_type.name <<
      " ; " << topic_with_type_and_count.topic_with_type.type <<
      " ; " << topic_with_type_and_count.message_count << "\n";
  }

  std::cout << ss.str() << std::endl;
}

}  // namespace rosbag2_transport
