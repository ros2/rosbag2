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

#include "rosbag2_transport/reader_writer_factory.hpp"

#include <memory>
#include <utility>

#include "rosbag2_compression/compression_options.hpp"
#include "rosbag2_compression/sequential_compression_reader.hpp"
#include "rosbag2_compression/sequential_compression_writer.hpp"
#include "rosbag2_storage/metadata_io.hpp"

namespace rosbag2_transport
{

std::unique_ptr<rosbag2_cpp::Reader> ReaderWriterFactory::make_reader(
  const rosbag2_storage::StorageOptions & storage_options)
{
  rosbag2_storage::MetadataIo metadata_io;
  std::unique_ptr<rosbag2_cpp::reader_interfaces::BaseReaderInterface> reader_impl;

  if (metadata_io.metadata_file_exists(storage_options.uri)) {
    auto metadata = metadata_io.read_metadata(storage_options.uri);
    if (!metadata.compression_format.empty()) {
      reader_impl = std::make_unique<rosbag2_compression::SequentialCompressionReader>();
    }
  }
  if (!reader_impl) {
    reader_impl = std::make_unique<rosbag2_cpp::readers::SequentialReader>();
  }

  return std::make_unique<rosbag2_cpp::Reader>(std::move(reader_impl));
}

std::unique_ptr<rosbag2_cpp::Writer> ReaderWriterFactory::make_writer(
  const rosbag2_transport::RecordOptions & record_options)
{
  std::unique_ptr<rosbag2_cpp::writer_interfaces::BaseWriterInterface> writer_impl;
  if (!record_options.compression_format.empty()) {
    rosbag2_compression::CompressionOptions compression_options {
      record_options.compression_format,
      rosbag2_compression::compression_mode_from_string(record_options.compression_mode),
      record_options.compression_queue_size,
      record_options.compression_threads
    };
    if (compression_options.compression_threads < 1) {
      compression_options.compression_threads = std::thread::hardware_concurrency();
    }
    writer_impl = std::make_unique<rosbag2_compression::SequentialCompressionWriter>(
      compression_options);
  } else {
    writer_impl = std::make_unique<rosbag2_cpp::writers::SequentialWriter>();
  }

  return std::make_unique<rosbag2_cpp::Writer>(std::move(writer_impl));
}

}  // namespace rosbag2_transport
