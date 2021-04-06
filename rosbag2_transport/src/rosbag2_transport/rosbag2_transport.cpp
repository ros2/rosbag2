// Copyright 2018, Bosch Software Innovations GmbH.
// Copyright 2020, TNG Technology Consulting GmbH.
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

#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "rcutils/time.h"

#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_cpp/typesupport_helpers.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_cpp/writers/sequential_writer.hpp"

#include "rosbag2_transport/logging.hpp"

#include "player.hpp"
#include "recorder.hpp"

namespace rosbag2_transport
{

Rosbag2Transport::Rosbag2Transport()
: reader_(std::make_shared<rosbag2_cpp::Reader>(
      std::make_unique<rosbag2_cpp::readers::SequentialReader>())),
  writer_(std::make_shared<rosbag2_cpp::Writer>(
      std::make_unique<rosbag2_cpp::writers::SequentialWriter>()))
{}

Rosbag2Transport::Rosbag2Transport(
  std::shared_ptr<rosbag2_cpp::Reader> reader,
  std::shared_ptr<rosbag2_cpp::Writer> writer)
: reader_(std::move(reader)), writer_(std::move(writer)) {}

void Rosbag2Transport::init()
{
  rclcpp::init(0, nullptr);
}

void Rosbag2Transport::shutdown()
{
  rclcpp::shutdown();
}

void Rosbag2Transport::record(
  const rosbag2_storage::StorageOptions & storage_options, const RecordOptions & record_options)
{
  try {
    writer_->open(
      storage_options, {rmw_get_serialization_format(), record_options.rmw_serialization_format});

    auto transport_node = setup_node(record_options.node_prefix);

    Recorder recorder(writer_, transport_node);
    recorder.record(record_options);
  } catch (std::runtime_error & e) {
    ROSBAG2_TRANSPORT_LOG_ERROR("Failed to record: %s", e.what());
  }
}

std::shared_ptr<rclcpp::Node> Rosbag2Transport::setup_node(
  std::string node_prefix,
  const std::vector<std::string> & topic_remapping_options)
{
  if (!transport_node_) {
    auto node_options = rclcpp::NodeOptions().arguments(topic_remapping_options);
    transport_node_ = std::make_shared<rclcpp::Node>(node_prefix + "_rosbag2", node_options);
  }
  return transport_node_;
}

void Rosbag2Transport::play(
  const rosbag2_storage::StorageOptions & storage_options, const PlayOptions & play_options)
{
  try {
    auto transport_node =
      setup_node(play_options.node_prefix, play_options.topic_remapping_options);
    Player player(reader_, transport_node);
    do {
      reader_->open(storage_options, {"", rmw_get_serialization_format()});
      player.play(play_options);
    } while (rclcpp::ok() && play_options.loop);
  } catch (std::runtime_error & e) {
    ROSBAG2_TRANSPORT_LOG_ERROR("Failed to play: %s", e.what());
  }
}

}  // namespace rosbag2_transport
