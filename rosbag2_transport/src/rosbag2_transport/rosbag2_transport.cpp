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

#include "rosbag2_transport/logging.hpp"
#include "rosbag2/sequential_reader.hpp"
#include "rosbag2/types.hpp"
#include "rosbag2/typesupport_helpers.hpp"
#include "rosbag2/writer.hpp"

#include "player.hpp"
#include "recorder.hpp"

namespace rosbag2_transport
{

Rosbag2Transport::Rosbag2Transport()
: reader_(std::make_shared<rosbag2::SequentialReader>()),
  writer_(std::make_shared<rosbag2::Writer>())
{}

Rosbag2Transport::Rosbag2Transport(
  std::shared_ptr<rosbag2::SequentialReader> reader, std::shared_ptr<rosbag2::Writer> writer)
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
  const StorageOptions & storage_options, const RecordOptions & record_options)
{
  writer_->open(storage_options);

  auto transport_node = setup_node();

  Recorder recorder(writer_, transport_node);
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
  reader_->open(storage_options);

  auto transport_node = setup_node();

  Player player(reader_, transport_node);
  player.play(play_options);
}

}  // namespace rosbag2_transport
