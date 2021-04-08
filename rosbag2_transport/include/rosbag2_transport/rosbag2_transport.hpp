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

#ifndef ROSBAG2_TRANSPORT__ROSBAG2_TRANSPORT_HPP_
#define ROSBAG2_TRANSPORT__ROSBAG2_TRANSPORT_HPP_

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rosbag2_storage/storage_options.hpp"

#include "rosbag2_transport/play_options.hpp"
#include "rosbag2_transport/record_options.hpp"
#include "rosbag2_transport/visibility_control.hpp"

namespace rosbag2_cpp
{
class Reader;
class Writer;
}  // namespace rosbag2_cpp

namespace rosbag2_transport
{

class Player
{
public:
  ROSBAG2_TRANSPORT_PUBLIC
  Player();

  ROSBAG2_TRANSPORT_PUBLIC
  explicit Player(std::shared_ptr<rosbag2_cpp::Reader> reader);

  ROSBAG2_TRANSPORT_PUBLIC
  virtual ~Player();

  /**
   * Replay a bagfile.
   *
   * \param storage_options Option regarding the storage (e.g. bag file name)
   * \param play_options Options regarding the playback (e.g. queue size)
   */
  ROSBAG2_TRANSPORT_PUBLIC
  void play(
    const rosbag2_storage::StorageOptions & storage_options,
    const PlayOptions & play_options);

protected:
  std::shared_ptr<rosbag2_cpp::Reader> reader_;
};

class Recorder
{
public:
  ROSBAG2_TRANSPORT_PUBLIC
  Recorder();

  ROSBAG2_TRANSPORT_PUBLIC
  explicit Recorder(std::shared_ptr<rosbag2_cpp::Writer> writer);

  ROSBAG2_TRANSPORT_PUBLIC
  virtual ~Recorder();

  /**
   * Records topics to a bagfile. Subscription happens at startup time, hence the topics must
   * exist when "record" is called.
   *
   * \param storage_options Options regarding the storage (e.g. bag file name)
   * \param record_options Options regarding the recording (e.g. the topics to record)
   */
  ROSBAG2_TRANSPORT_PUBLIC
  void record(
    const rosbag2_storage::StorageOptions & storage_options,
    const RecordOptions & record_options);

protected:
  std::shared_ptr<rosbag2_cpp::Writer> writer_;
};

}  // namespace rosbag2_transport

#endif  // ROSBAG2_TRANSPORT__ROSBAG2_TRANSPORT_HPP_
