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

#ifndef ROSBAG2_TRANSPORT__PLAYER_HPP_
#define ROSBAG2_TRANSPORT__PLAYER_HPP_

#include <future>
#include <map>
#include <memory>
#include <queue>
#include <string>

#include "moodycamel/readerwriterqueue.h"
#include "replayable_message.hpp"
#include "rosbag2/sequential_reader.hpp"
#include "rosbag2/types.hpp"
#include "rosbag2_transport/play_options.hpp"

using TimePoint = std::chrono::time_point<std::chrono::high_resolution_clock>;

namespace rosbag2_transport
{

class GenericPublisher;
class Rosbag2Node;

class Player
{
public:
  explicit Player(std::shared_ptr<rosbag2::SequentialReader> reader);

  void play(const PlayOptions & options);

private:
  void load_storage_content(const PlayOptions & options);
  bool is_storage_completely_loaded() const;
  void enqueue_up_to_boundary(const TimePoint & time_first_message, uint64_t boundary);
  void wait_for_filled_queue(const PlayOptions & options) const;
  void play_messages_from_queue();
  void prepare_publishers();

  static constexpr double read_ahead_lower_bound_percentage_ = 0.9;

  std::shared_ptr<rosbag2::SequentialReader> reader_;
  moodycamel::ReaderWriterQueue<ReplayableMessage> message_queue_;
  mutable std::future<void> storage_loading_future_;
  std::shared_ptr<Rosbag2Node> node_;
  std::map<std::string, std::shared_ptr<GenericPublisher>> publishers_;
};

}  // namespace rosbag2_transport

#endif  // ROSBAG2_TRANSPORT__PLAYER_HPP_
