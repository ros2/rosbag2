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

#ifndef ROSBAG2_CPP__READERS__RANDOM_ACCESS_READER_HPP_
#define ROSBAG2_CPP__READERS__RANDOM_ACCESS_READER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rosbag2_cpp/readers/sequential_reader.hpp"

// This is necessary because of using stl types here. It is completely safe, because
// a) the member is not accessible from the outside
// b) there are no inline functions.
#ifdef _WIN32
# pragma warning(push)
# pragma warning(disable:4251)
#endif

namespace rosbag2_cpp
{
namespace readers
{

class ROSBAG2_CPP_PUBLIC RandomAccessReader
  : public ::rosbag2_cpp::readers::SequentialReader
{
public:
  RandomAccessReader(
    std::unique_ptr<rosbag2_storage::StorageFactoryInterface> storage_factory =
    std::make_unique<rosbag2_storage::StorageFactory>(),
    std::shared_ptr<SerializationFormatConverterFactoryInterface> converter_factory =
    std::make_shared<SerializationFormatConverterFactory>(),
    std::unique_ptr<rosbag2_storage::MetadataIo> metadata_io =
    std::make_unique<rosbag2_storage::MetadataIo>())
    : SequentialReader(std::move(storage_factory), std::move(converter_factory), std::move(metadata_io)) {}

  virtual ~RandomAccessReader() {}

  std::shared_ptr<rosbag2_storage::SerializedBagMessage> read_at_timestamp(rcutils_time_point_value_t timestamp);

  std::shared_ptr<rosbag2_storage::SerializedBagMessage> read_at_index(int index); // TODO: CHANGE TYPE
};

}  // namespace readers
}  // namespace rosbag2_cpp

#ifdef _WIN32
# pragma warning(pop)
#endif

#endif  // ROSBAG2_CPP__READERS__RANDOM_ACCESS_READER_HPP_
