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

#ifndef ROSBAG2__SEQUENTIAL_READER_HPP_
#define ROSBAG2__SEQUENTIAL_READER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rosbag2_storage/storage_factory.hpp"
#include "rosbag2_storage/storage_interfaces/read_only_interface.hpp"
#include "rosbag2/storage_options.hpp"
#include "rosbag2/types.hpp"
#include "rosbag2/visibility_control.hpp"

namespace rosbag2
{
/**
 * The SequentialReader allows opening and reading messages of a bag. Messages will be read
 * sequentially according to timestamp.
 */
class SequentialReader
{
public:
  /// Destructor
  ROSBAG2_PUBLIC
  virtual ~SequentialReader();

  /**
   * Open a rosbag for reading messages sequentially (time-ordered). Throws if file could not be
   * opened.
   *
   * \param options Options to configure the storage
   */
  ROSBAG2_PUBLIC
  virtual void open(const StorageOptions & options);

  /**
   * Ask whether the underlying bagfile contains at least one more message.
   *
   * \return true if storage contains at least one more message
   * \throws runtime_error Reader is not open.
   */
  ROSBAG2_PUBLIC
  virtual bool has_next();

  /**
   * Read next message from storage. Will throw if no more messages are available.
   *
   * Expected usage:
   * if (writer.has_next()) message = writer.read_next();
   *
   * \return next message in serialized form
   * \throws runtime_error Reader is not open.
   */
  ROSBAG2_PUBLIC
  virtual std::shared_ptr<SerializedBagMessage> read_next();

  /**
   * Ask bagfile for all topics (including their type identifier) that were recorded.
   *
   * \return vector of topics with topic name and type as std::string
   * \throws runtime_error Reader is not open.
   */
  ROSBAG2_PUBLIC
  virtual std::vector<TopicWithType> get_all_topics_and_types();

private:
  rosbag2_storage::StorageFactory factory_;
  std::shared_ptr<rosbag2_storage::storage_interfaces::ReadOnlyInterface> storage_;
};

}  // namespace rosbag2

#endif  // ROSBAG2__SEQUENTIAL_READER_HPP_
