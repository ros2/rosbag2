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

#include "../../src/rosbag2/reader.hpp"

namespace rosbag2
{
/**
 * The SequentialReader allows opening and reading messages of a bag with multiple database files.
 * Messages will be read sequentially according to timestamp.
 */
class ROSBAG2_PUBLIC SequentialReader
{
public:
  explicit
  SequentialReader(
    std::unique_ptr<rosbag2_storage::StorageFactoryInterface> storage_factory =
    std::make_unique<rosbag2_storage::StorageFactory>(),
    std::shared_ptr<SerializationFormatConverterFactoryInterface> converter_factory =
    std::make_shared<SerializationFormatConverterFactory>()) - ;

  ~SequentialReader() = default;

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
    const StorageOptions & storage_options, const ConverterOptions & converter_options);

  /**
   * Ask whether the underlying bagfile contains at least one more message. If not, checks if there's
   * another file to read from and if so, update the storage factory to read from it.
   *
   * \return true if storage contains at least one more message
   * \throws runtime_error if the Reader is not open.
   */
  virtual bool has_next();

  /**
   * Ask bagfile for all topics (including their type identifier) that were recorded.
   *
   * \return vector of topics with topic name and type as std::string
   * \throws runtime_error if the Reader is not open.
   */
  virtual std::vector<TopicMetadata> get_all_topics_and_types();

  /**
   * Read next message from storage. Will throw if no more messages are available.
   * The message will be serialized in the format given to `open`.
   *
   * Expected usage:
   * if (reader.has_next()) message = reader.read_next();
   *
   * \return next message in serialized form
   * \throws runtime_error if the Reader is not open.
   */
  virtual std::shared_ptr<SerializedBagMessage> read_next();

private:
  /**
   * Ask whether there is another database file to read from the list of relative file paths.
   *
   * \return true if there are still files to read in the list. false otherwise.
   */
  bool has_next_file() const;

  /**
   * Update the current database file to read from by incrementing the current file offset.
   * A std::out_of_range error will be thrown if the file offset is out of range.
   *
   * Expected usage:
   * if (has_next_file_()) file_string = get_next_file_();
   *
   * \throws runtime_error if the list of file paths is empty.
   */
  std::string get_next_file();

  std::shared_ptr<rosbag2::Reader> reader_;  // Underlying reader_ class that reads single bags.
  StorageOptions storage_options_;  // The storage options passed to the reader_.
  std::vector<std::string> file_paths_ {};  // List of database files.
  std::size_t current_file_offset_ {0};  // Index of file to read from file_paths_.
};

}  // namespace rosbag2

#endif  // ROSBAG2__SEQUENTIAL_READER_HPP_
