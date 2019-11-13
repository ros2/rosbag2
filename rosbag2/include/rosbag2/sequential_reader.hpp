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

#include "rosbag2_storage/metadata_io.hpp"
#include "rosbag2_storage/storage_factory.hpp"
#include "rosbag2_storage/storage_factory_interface.hpp"
#include "rosbag2_storage/storage_interfaces/read_only_interface.hpp"
#include "rosbag2/converter.hpp"
#include "rosbag2/serialization_format_converter_factory.hpp"
#include "rosbag2/serialization_format_converter_factory_interface.hpp"
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
 * sequentially according to timestamp. Functions are virtual so that they can be mocked in tests.
 */
class ROSBAG2_PUBLIC SequentialReader
{
public:
  explicit
  SequentialReader(
    std::unique_ptr<rosbag2_storage::StorageFactoryInterface> storage_factory =
    std::make_unique<rosbag2_storage::StorageFactory>(),
    std::shared_ptr<SerializationFormatConverterFactoryInterface> converter_factory =
    std::make_shared<SerializationFormatConverterFactory>(),
    std::unique_ptr<rosbag2_storage::MetadataIo> metadata_io =
    std::make_unique<rosbag2_storage::MetadataIo>());

  virtual ~SequentialReader();

  /**
   * Open a rosbag for reading messages sequentially (time-ordered). Throws if file could not be
   * opened. This must be called before any other function is used. The rosbag is
   * automatically closed on destruction.
   *
   * If the `output_serialization_format` within the `converter_options` is not the same as the
   * format of the underlying stored data, a converter will be used to automatically convert the
   * data to the specified output format.
   * Throws if the converter plugin does not exist.
   *
   * \param storage_options Options to configure the storage
   * \param converter_options Options for specifying the output data format
   */
  virtual void open(
    const StorageOptions & storage_options,
    const ConverterOptions & converter_options);

  /**
   * Ask whether the underlying bagfile contains at least one more message.
   *
   * \return true if storage contains at least one more message
   * \throws runtime_error if the Reader is not open.
   */
  virtual bool has_next();

  /**
   * Read next message from storage. Will throw if no more messages are available.
   * The message will be serialized in the format given to `open`.
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
  virtual std::vector<TopicMetadata> get_all_topics_and_types();

  /**
   * Ask whether there is another database file to read from the list of relative
   * file paths.
   *
   * \return true if there are still files to read in the list
   */
  virtual bool has_next_file() const;

  /**
   * Increment the current file iterator to point to the next file in the list of relative file
   * paths.
   *
   * Expected usage:
   * if (has_next_file()) load_next_file();
   */
  virtual void load_next_file();

  /**
   * Return the relative file path pointed to by the current file iterator.
   */
  virtual std::string get_current_file() const;

  /**
   * Return the URI of the current file (i.e. no extensions).
   */
  virtual std::string get_current_uri() const;

private:
  std::unique_ptr<rosbag2_storage::StorageFactoryInterface> storage_factory_;
  std::shared_ptr<SerializationFormatConverterFactoryInterface> converter_factory_;
  std::shared_ptr<rosbag2_storage::storage_interfaces::ReadOnlyInterface> storage_;
  std::unique_ptr<rosbag2_storage::MetadataIo> metadata_io_;
  rosbag2_storage::BagMetadata metadata_;
  std::unique_ptr<Converter> converter_;
  StorageOptions storage_options_{};
  std::vector<std::string> file_paths_{};  // List of database files.
  std::vector<std::string>::iterator current_file_iterator_{};  // Index of file to read from
};

}  // namespace rosbag2

#ifdef _WIN32
# pragma warning(pop)
#endif

#endif  // ROSBAG2__SEQUENTIAL_READER_HPP_
