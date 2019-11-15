// Copyright 2018, Bosch Software Innovations GmbH.
// Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#ifndef ROSBAG2__READER_HPP_
#define ROSBAG2__READER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rosbag2/converter_options.hpp"
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
namespace reader_interfaces
{
class BaseReaderInterface;
}  // namespace reader_interfaces

/**
 * The Reader allows opening and reading messages of a bag.
 */
class ROSBAG2_PUBLIC Reader final
{
public:
  explicit Reader(std::unique_ptr<reader_interfaces::BaseReaderInterface> reader_impl);

  ~Reader();

  /**
   * Throws if file could not be opened.
   * This must be called before any other function is used.
   * The rosbag is automatically closed on destruction.
   *
   * If the `output_serialization_format` within the `converter_options` is not the same as the
   * format of the underlying stored data, a converter will be used to automatically convert the
   * data to the specified output format.
   * Throws if the converter plugin does not exist.
   *
   * \param storage_options Options to configure the storage
   * \param converter_options Options for specifying the output data format
   */
  void open(
    const StorageOptions & storage_options, const ConverterOptions & converter_options);

  /**
   * Ask whether the underlying bagfile contains at least one more message.
   *
   * \return true if storage contains at least one more message
   * \throws runtime_error if the Reader is not open.
   */
  bool has_next();

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
  std::shared_ptr<SerializedBagMessage> read_next();

  /**
   * Ask bagfile for all topics (including their type identifier) that were recorded.
   *
   * \return vector of topics with topic name and type as std::string
   * \throws runtime_error if the Reader is not open.
   */
  std::vector<TopicMetadata> get_all_topics_and_types();

  reader_interfaces::BaseReaderInterface & get_implementation_handle() const
  {
    return *reader_impl_;
  }

private:
  std::unique_ptr<reader_interfaces::BaseReaderInterface> reader_impl_;
};

}  // namespace rosbag2

#ifdef _WIN32
# pragma warning(pop)
#endif

#endif  // ROSBAG2__READER_HPP_
