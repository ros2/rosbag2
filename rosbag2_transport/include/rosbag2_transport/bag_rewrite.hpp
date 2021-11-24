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

#ifndef ROSBAG2_TRANSPORT__BAG_REWRITE_HPP_
#define ROSBAG2_TRANSPORT__BAG_REWRITE_HPP_

#include <memory>
#include <utility>
#include <vector>

#include "rosbag2_storage/storage_options.hpp"
#include "rosbag2_transport/record_options.hpp"

namespace rosbag2_transport
{
/// Given one or more existing bags, write out one or more new bags with new settings.
/// This generic feature enables (but is not limited to) the following features:
/// - merge (multiple input bags, one output bag)
/// - split (one input bag, one output bag with some size or duration splitting values)
/// - filter (input bag(s) - output bag(s) accept different topics)
/// - compress
/// - serialization format conversion
///
/// \param input_options vector of settings to create Readers for bags to read messages from
/// \param output_bags - full "recording" configuration of the bag(s) to write messages to
///   Each output bag will be passed messages from every input bag,
///   on topics that pass its filtering settings
void bag_rewrite(
  const std::vector<rosbag2_storage::StorageOptions> & input_options,
  const std::vector<
    std::pair<rosbag2_storage::StorageOptions, rosbag2_transport::RecordOptions>
  > & output_options
);
}  // namespace rosbag2_transport

#endif  // ROSBAG2_TRANSPORT__BAG_REWRITE_HPP_
