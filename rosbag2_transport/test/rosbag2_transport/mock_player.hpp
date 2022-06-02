// Copyright 2021, Apex.AI
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

#ifndef ROSBAG2_TRANSPORT__MOCK_PLAYER_HPP_
#define ROSBAG2_TRANSPORT__MOCK_PLAYER_HPP_

#include <memory>
#include <utility>
#include <vector>

#include "rosbag2_transport/player.hpp"

class MockPlayer : public rosbag2_transport::Player
{
public:
  MockPlayer(
    std::unique_ptr<rosbag2_cpp::Reader> reader,
    const rosbag2_storage::StorageOptions & storage_options,
    const rosbag2_transport::PlayOptions & play_options)
  : Player(std::move(reader), storage_options, play_options)
  {}

  std::vector<rclcpp::PublisherBase *> get_list_of_publishers()
  {
    std::vector<rclcpp::PublisherBase *> pub_list;
    for (const auto & publisher : publishers_) {
      pub_list.push_back(
        static_cast<rclcpp::PublisherBase *>(
          publisher.second->generic_publisher().get()));
    }
    return pub_list;
  }

  void wait_for_playback_to_start()
  {
    std::unique_lock<std::mutex> lk(ready_to_play_from_queue_mutex_);
    ready_to_play_from_queue_cv_.wait(lk, [this] {return is_ready_to_play_from_queue_;});
  }

  size_t get_number_of_registered_pre_callbacks()
  {
    size_t callback_counter = 0;
    std::lock_guard<std::mutex> lk(on_play_msg_callbacks_mutex_);
    for (auto & pre_callback_data : on_play_msg_pre_callbacks_) {
      (void)pre_callback_data;
      callback_counter++;
    }
    return callback_counter;
  }

  size_t get_number_of_registered_post_callbacks()
  {
    size_t callback_counter = 0;
    std::lock_guard<std::mutex> lk(on_play_msg_callbacks_mutex_);
    for (auto & post_callback_data : on_play_msg_post_callbacks_) {
      (void)post_callback_data;
      callback_counter++;
    }
    return callback_counter;
  }
};

#endif  // ROSBAG2_TRANSPORT__MOCK_PLAYER_HPP_
