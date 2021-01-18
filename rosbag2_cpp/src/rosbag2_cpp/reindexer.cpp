// Copyright 2020 DCS Corporation, All Rights Reserved.
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
//
// DISTRIBUTION A. Approved for public release; distribution unlimited.
// OPSEC #4584.
//
// Delivered to the U.S. Government with Unlimited Rights, as defined in DFARS
// Part 252.227-7013 or 7014 (Feb 2014).
//
// This notice must appear in all copies of this file and its derivatives.

#include "rosbag2_cpp/reindexer.hpp"

#include <memory>
#include <utility>
#include <vector>

#include "rosbag2_cpp/storage_options.hpp"
#include "rosbag2_cpp/info.hpp"
#include "rosbag2_cpp/reindexer_interfaces/base_reindexer_interface.hpp"

namespace rosbag2_cpp
{

Reindexer::Reindexer(std::unique_ptr<reindexer_interfaces::BaseReindexerInterface> reindexer_impl)
: reindexer_impl(std::move(reindexer_impl))
{}

Reindexer::~Reindexer() {}

void Reindexer::reindex(const StorageOptions & storage_options)
{
  reindexer_impl->reindex(storage_options);
}


}  // namespace rosbag2_cpp
