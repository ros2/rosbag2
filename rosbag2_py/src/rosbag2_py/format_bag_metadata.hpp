// Copyright 2018-2021, Bosch Software Innovations GmbH.
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

#ifndef ROSBAG2_PY__FORMAT_BAG_METADATA_HPP_
#define ROSBAG2_PY__FORMAT_BAG_METADATA_HPP_

#include <string>

#include "rosbag2_storage/bag_metadata.hpp"

std::string format_bag_meta_data(const rosbag2_storage::BagMetadata & metadata);

#endif  // ROSBAG2_PY__FORMAT_BAG_METADATA_HPP_
