// Copyright 2021 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#ifndef ROSBAG2_TRANSPORT__READER_WRITER_FACTORY_HPP_
#define ROSBAG2_TRANSPORT__READER_WRITER_FACTORY_HPP_

#include <memory>

#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "rosbag2_transport/record_options.hpp"
#include "rosbag2_transport/visibility_control.hpp"

namespace rosbag2_transport
{
class ROSBAG2_TRANSPORT_PUBLIC ReaderWriterFactory
{
public:
  /// Create a Reader with the appropriate underlying implementation.
  static std::unique_ptr<rosbag2_cpp::Reader> make_reader(
    const rosbag2_storage::StorageOptions & storage_options);

  /// Create a Writer with the appropriate underlying implementation.
  static std::unique_ptr<rosbag2_cpp::Writer> make_writer(
    const rosbag2_transport::RecordOptions & record_options);
};
}  // namespace rosbag2_transport

#endif  // ROSBAG2_TRANSPORT__READER_WRITER_FACTORY_HPP_
