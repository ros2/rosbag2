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

#include <string>

#include "pluginlib/class_list_macros.hpp"

#include "fake_compressor.hpp"

std::string FakeCompressor::compress_uri(const std::string & uri)
{
  return uri + "." + get_compression_identifier();
}

void FakeCompressor::compress_serialized_bag_message(
  const rosbag2_storage::SerializedBagMessage *,
  rosbag2_storage::SerializedBagMessage *) {}

std::string FakeCompressor::get_compression_identifier() const
{
  return "fake_comp";
}

PLUGINLIB_EXPORT_CLASS(FakeCompressor, rosbag2_compression::BaseCompressorInterface)
