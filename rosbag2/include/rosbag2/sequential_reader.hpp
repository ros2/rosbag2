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

// This is necessary because of using stl types here. It is completely safe, because
// a) the member is not accessible from the outside
// b) there are no inline functions.
#ifdef _WIN32
# pragma warning(push)
# pragma warning(disable:4251)
#endif

namespace rosbag2
{
/**
 * The SequentialReader allows opening and reading messages of a bag. Messages will be read
 * sequentially according to timestamp.
 */
class ROSBAG2_PUBLIC SequentialReader
{
public:
  virtual ~SequentialReader();

  /**
   * Open a rosbag for reading messages sequentially (time-ordered). Throws if file could not be
   * opened. This must be called before any other function is used. The rosbag is
   * automatically closed on destruction.
   *
   * \param options Options to configure the storage
   */
  virtual void open(const StorageOptions & options);

  /**
   * Ask whether the underlying bagfile contains at least one more message.
   *
   * \return true if storage contains at least one more message
   * \throws runtime_error if the Reader is not open.
   */
  virtual bool has_next();

  /**
   * Read next message from storage. Will throw if no more messages are available.
   *
   * Expected usage:
   * if (writer.has_next()) message = writer.read_next();
   *
   * \return next message in serialized form
   * \throws runtime_error if the Reader is not open.
   */
  virtual std::shared_ptr<SerializedBagMessage> read_next();

  /**
   * Ask bagfile for all topics (including their type identifier) that were recorded.
   *
   * \return vector of topics with topic name and type as std::string
   * \throws runtime_error if the Reader is not open.
   */
  virtual std::vector<TopicWithType> get_all_topics_and_types();

  /**
   * Retrieves the metadata stored in the yaml file relative to the opened database.
   *
   * \param uri the path to the directory where the yaml file is.
   *
   * \return rosbag2_storage::BagMetadata struct relative to the database associated with the Reader
   * \throws runtime_error if the Reader is not open.
   */
  virtual rosbag2_storage::BagMetadata info(const std::string & uri);

private:
  rosbag2_storage::StorageFactory factory_;
  std::shared_ptr<rosbag2_storage::storage_interfaces::ReadOnlyInterface> storage_;
};

}  // namespace rosbag2

#endif  // ROSBAG2__SEQUENTIAL_READER_HPP_
