// Copyright 2022, Apex.AI
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

#ifndef ROSBAG2_TRANSPORT__MOCK_RECORDER_HPP_
#define ROSBAG2_TRANSPORT__MOCK_RECORDER_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <ratio>

#include "rosbag2_transport/recorder.hpp"

class MockRecorder : public rosbag2_transport::Recorder
{
public:
  MockRecorder(
    std::shared_ptr<rosbag2_cpp::Writer> writer,
    const rosbag2_storage::StorageOptions & storage_options,
    const rosbag2_transport::RecordOptions & record_options)
  : Recorder(writer, storage_options, record_options)
  {}

  template<typename DurationRepT = int64_t, typename DurationT = std::milli>
  bool wait_for_topic_to_be_discovered(
    const std::string & topic_name_to_wait_for,
    std::chrono::duration<DurationRepT, DurationT> timeout = std::chrono::seconds(10))
  {
    bool discovered = false;
    using clock = std::chrono::steady_clock;
    auto start = clock::now();
    do {
      auto topic_names_and_types = this->get_topic_names_and_types();
      for (const auto &[topic_name, topic_types] : topic_names_and_types) {
        if (topic_name_to_wait_for == topic_name) {
          discovered = true;
          break;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    } while (!discovered && (clock::now() - start) < timeout);
    return discovered;
  }

  bool topic_available_for_recording(const std::string & topic_name)
  {
    bool available_for_recording = false;
    auto topics_to_subscribe = this->get_requested_or_available_topics();
    for (const auto & topic_and_type : topics_to_subscribe) {
      if (topic_and_type.first == topic_name) {
        available_for_recording = true;
        break;
      }
    }
    return available_for_recording;
  }
};

#endif  // ROSBAG2_TRANSPORT__MOCK_RECORDER_HPP_
