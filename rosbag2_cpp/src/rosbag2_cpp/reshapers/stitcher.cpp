// Copyright 2018, Bosch Software Innovations GmbH.
// Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
// Copyright 2021 Firefly Automatix, Inc.
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

#include "rosbag2_cpp/reshapers/stitcher.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rosbag2_cpp/reader_interfaces/base_reader_interface.hpp"
#include "rosbag2_cpp/writer_interfaces/base_writer_interface.hpp"

namespace rosbag2_cpp
{

static constexpr char const * kDefaultStorageID = "sqlite3";

Stitcher::Stitcher(
std::unique_ptr<reader_interfaces::BaseReaderInterface> reader_impl,
std::unique_ptr<writer_interfaces::BaseWriterInterface> writer_impl)
: reader_impl_(std::move(reader_impl)), writer_impl_(std::move(writer_impl))
{}

Stitcher::~Stitcher()
{
  reader_impl_->reset();
  writer_impl_->reset();
}

void Stitcher::open(
  const std::vector<std::string> & storage_uris,
  const std::string & output_uri)
{
  rosbag2_storage::StorageOptions writer_storage_options;
  writer_storage_options.uri = output_uri;
  writer_storage_options.storage_id = kDefaultStorageID;
  rosbag2_cpp::ConverterOptions converter_options{};
  
  return open(storage_uris, writer_storage_options, converter_options);
}

void Stitcher::open(
  const std::vector<std::string> & storage_uris,
  const rosbag2_storage::StorageOptions & writer_storage_options,
  const ConverterOptions & converter_options)
{
  // open writer object for output
  writer_impl_->open(writer_storage_options, converter_options);

  // store default converter, storage options
  converter_options_ = converter_options;
  storage_id_ = writer_storage_options.storage_id;

  // store uris to read for later use
  storage_uris_ = std::move(storage_uris);
  current_iter_ = storage_uris_.begin();
}

bool Stitcher::has_next()
{
  return std::distance(storage_uris_.end(), current_iter_) != 0;
}

void Stitcher::stitch_next()
{
  // read bag from current iterator
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = *current_iter_;
  storage_options.storage_id = storage_id_;
  reader_impl_->open(storage_options, converter_options_);

  // write contents of bag to output writer
  while (reader_impl_->has_next()) {
    auto bag_message = reader_impl_->read_next();
    writer_impl_->write(bag_message);
  }

  // advance iterator
  current_iter_++;

  // clear reader impl to get ready for next iteration
  reader_impl_->reset();
}

void Stitcher::set_filter(const rosbag2_storage::StorageFilter & storage_filter)
{
  reader_impl_->set_filter(storage_filter);
}

void Stitcher::reset_filter()
{
  reader_impl_->reset_filter();
}

}  // namespace rosbag2_cpp