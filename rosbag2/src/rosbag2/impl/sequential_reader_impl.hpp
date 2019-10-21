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

#ifndef ROSBAG2__IMPL__SEQUENTIAL_READER_IMPL_HPP_
#define ROSBAG2__IMPL__SEQUENTIAL_READER_IMPL_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rosbag2/sequential_reader.hpp"

namespace rosbag2
{

class ROSBAG2_LOCAL ReaderImpl
{
public:
  ReaderImpl(
    std::unique_ptr<rosbag2_storage::StorageFactoryInterface> storage_factory,
    std::shared_ptr<SerializationFormatConverterFactoryInterface> converter_factory);

  ~ReaderImpl();
  void reset();
  void open(
    const StorageOptions & storage_options, const ConverterOptions & converter_options);
  bool has_next();
  std::shared_ptr<SerializedBagMessage> read_next();
  std::vector<TopicMetadata> get_all_topics_and_types();

private:
  /**
   * Checks if all topics in the bagfile have the same RMW serialization format.
   * Currently a bag file can only be played if all topics have the same serialization format.
   *
   * \param topics Vector of TopicInformation with metadata.
   * \throws runtime_error if any topic has a different serialization format from the rest.
   */
  virtual void check_topics_serialization_formats(const std::vector<TopicInformation> & topics);

  /**
   * Checks if the serialization format of the converter factory is the same as that of the storage
   * factory.
   * If not, changes the serialization format of the converter factory to use the serialization
   * format of the storage factory.
   *
   * \param converter_serialization_format
   * \param storage_serialization_format
   */
  virtual void check_converter_serialization_format(
    const std::string & converter_serialization_format,
    const std::string & storage_serialization_format);

  std::unique_ptr<rosbag2_storage::StorageFactoryInterface> storage_factory_{};
  std::shared_ptr<SerializationFormatConverterFactoryInterface> converter_factory_{};
  std::shared_ptr<rosbag2_storage::storage_interfaces::ReadOnlyInterface> storage_{};
  std::unique_ptr<Converter> converter_{};
};
}  // namespace rosbag2

#endif  // ROSBAG2__IMPL__SEQUENTIAL_READER_IMPL_HPP_
