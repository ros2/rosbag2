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

#ifndef ROSBAG2__WRITER_IMPL_HPP_
#define ROSBAG2__WRITER_IMPL_HPP_

#include <memory>
#include <string>

#include "rosbag2_storage/storage_factory.hpp"
#include "rosbag2_storage/storage_interfaces/read_write_interface.hpp"
#include "rosbag2_storage/metadata_io_iface.hpp"
#include "rosbag2/storage_options.hpp"
#include "rosbag2/types.hpp"
#include "rosbag2/visibility_control.hpp"
#include "rosbag2/writer.hpp"

namespace rosbag2
{

/**
 * The Writer allows writing messages to a new bag. For every topic, information about its type
 * needs to be added before writing the first message.
 */
class WriterImpl : public Writer
{
public:
  ~WriterImpl() override;

  /**
   * Opens a new bagfile and prepare it for writing messages. The bagfile must not exist.
   * This must be called before any other function is used.
   *
   * \param options Options to configure the storage
   */
  void open(const StorageOptions & options) override;

  /**
   * Create a new topic in the underlying storage. Needs to be called for every topic used within
   * a message which is passed to write(...).
   *
   * \param topic_with_type name and type identifier of topic to be created
   * \throws runtime_error if the Writer is not open.
   */
  void create_topic(const TopicWithType & topic_with_type) override;

  /**
   * Write a message to a bagfile. The topic needs to have been created before writing is possible.
   *
   * \param message to be written to the bagfile
   * \throws runtime_error if the Writer is not open.
   */
  void write(std::shared_ptr<SerializedBagMessage> message) override;

private:
  rosbag2::StorageOptions options_;
  rosbag2_storage::StorageFactory factory_;
  std::shared_ptr<rosbag2_storage::storage_interfaces::ReadWriteInterface> storage_;
  std::unique_ptr<rosbag2_storage::MetadataIOIface> metadata_io_;
};

}  // namespace rosbag2

#endif  // ROSBAG2__WRITER_IMPL_HPP_
