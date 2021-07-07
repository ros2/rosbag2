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

#include "rosbag2_cpp/rewriter.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/writer.hpp"

namespace rosbag2_cpp
{

Rewriter::Rewriter(
  std::unique_ptr<Reader> reader,
  std::unique_ptr<Writer> writer)
: reader_(std::move(reader)), writer_(std::move(writer))
{}

Rewriter::~Rewriter()
{}

void Rewriter::rewrite(
  rosbag2_storage::StorageOptions input_storage_options,
  ConverterOptions input_converter_options,
  rosbag2_storage::StorageOptions output_storage_options,
  ConverterOptions output_converter_options)
{
  reader_->open(input_storage_options, input_converter_options);
  writer_->open(output_storage_options, output_converter_options);

  while (reader_->has_next()) {
    auto msg = reader_->read_next();
    writer_->write(msg);
  }
}

void Rewriter::rewrite_many(
  std::vector<rosbag2_storage::StorageOptions> input_storage_options,
  ConverterOptions input_converter_options,
  rosbag2_storage::StorageOptions output_storage_options,
  ConverterOptions output_converter_options)
{
  writer_->open(output_storage_options, output_converter_options);

  for (auto & storage : input_storage_options) {
    reader_->open(storage, input_converter_options);

    while (reader_->has_next()) {
      auto msg = reader_->read_next();
      writer_->write(msg);
    }
  }
}

}  // namespace rosbag2_cpp
