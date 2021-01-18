// DISTRIBUTION A. Approved for public release; distribution unlimited.
// OPSEC #4584.
//
// Copyright (c) 2020 DCS Corporation
// All Rights Reserved
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    Caution-http://www.apache.org/licenses/LICENSE-2.0
//
// The software/firmware is provided to you on an As-Is basis.
//
// Delivered to the U.S. Government with Unlimited Rights, as defined in DFARS
// Part 252.227-7013 or 7014 (Feb 2014).
//
// This notice must appear in all copies of this file and its derivatives.

#ifndef ROSBAG2_CPP__REINDEXER_INTERFACES__BASE_REINDEXER_INTERFACE_HPP_
#define ROSBAG2_CPP__REINDEXER_INTERFACES__BASE_REINDEXER_INTERFACE_HPP_

#include <memory>
#include <vector>

#include "rosbag2_cpp/converter_options.hpp"
#include "rosbag2_cpp/storage_options.hpp"
#include "rosbag2_cpp/visibility_control.hpp"

#include "rosbag2_storage/bag_metadata.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"
#include "rosbag2_storage/storage_filter.hpp"
#include "rosbag2_storage/topic_metadata.hpp"

namespace rosbag2_cpp
{
namespace reindexer_interfaces
{

class ROSBAG2_CPP_PUBLIC BaseReindexerInterface
{
public:
  virtual ~BaseReindexerInterface() {}

  virtual void reindex(const StorageOptions & storage_options) = 0;
};

}  // namespace reindexer_interfaces
}  // namespace rosbag2_cpp

#endif  // ROSBAG2_CPP__REINDEXER_INTERFACES__BASE_REINDEXER_INTERFACE_HPP_
