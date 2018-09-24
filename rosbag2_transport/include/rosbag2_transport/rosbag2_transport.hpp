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
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rosbag2/sequential_reader.hpp"
#include "rosbag2/writer.hpp"
#include "rosbag2_transport/play_options.hpp"
#include "rosbag2_transport/record_options.hpp"
#include "rosbag2_transport/storage_options.hpp"
#include "rosbag2_transport/visibility_control.hpp"

namespace rosbag2_transport
{

class GenericPublisher;
class GenericSubscription;
class Rosbag2Node;
class Player;

class Rosbag2Transport
{
public:
  /// Constructor
  ROSBAG2_TRANSPORT_PUBLIC
  explicit Rosbag2Transport(
    std::shared_ptr<rosbag2::SequentialReader> reader =
    std::make_shared<rosbag2::SequentialReader>(),
    std::shared_ptr<rosbag2::Writer> writer = rosbag2::create_default_writer());

  ROSBAG2_TRANSPORT_PUBLIC
  void init();

  ROSBAG2_TRANSPORT_PUBLIC
  void shutdown();

  /**
   * Records topics to a bagfile. Subscription happens at startup time, hence the topics must
   * exist when "record" is called.
   *
   * \param storage_options Options regarding the storage (e.g. bag file name)
   * \param record_options Options regarding the recording (e.g. the topics to record)
   */
  ROSBAG2_TRANSPORT_PUBLIC
  void record(const StorageOptions & storage_options, const RecordOptions & record_options);

  /**
   * Replay all topics in a bagfile.
   *
   * \param storage_options Option regarding the storage (e.g. bag file name)
   * \param play_options Options regarding the playback (e.g. queue size)
   */
  ROSBAG2_TRANSPORT_PUBLIC
  void play(const StorageOptions & storage_options, const PlayOptions & play_options);

  /**
   * Print the bag info contained in the metadata yaml file.
   *
   * \param uri path to the metadata yaml file.
   */
  ROSBAG2_TRANSPORT_PUBLIC
  void print_bag_info(const std::string & uri);

private:
  std::shared_ptr<rosbag2_transport::GenericSubscription>
  create_subscription(
    std::shared_ptr<Rosbag2Node> & node,
    const std::string & topic_name, const std::string & topic_type) const;

  std::shared_ptr<rosbag2::SequentialReader> reader_;
  std::shared_ptr<rosbag2::Writer> writer_;

  std::shared_ptr<Rosbag2Node> transport_node_;

  std::vector<std::shared_ptr<GenericSubscription>> subscriptions_;

  std::shared_ptr<Rosbag2Node> setup_node();
};

}  // namespace rosbag2_transport

#endif  // ROSBAG2_TRANSPORT__ROSBAG2_TRANSPORT_HPP_
