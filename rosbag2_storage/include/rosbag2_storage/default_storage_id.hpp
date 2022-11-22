// Copyright 2022, Foxglove Technologies Inc. All rights reserved.
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

#ifndef ROSBAG2_STORAGE__DEFAULT_STORAGE_ID_HPP_
#define ROSBAG2_STORAGE__DEFAULT_STORAGE_ID_HPP_

#include <string>

#include "rosbag2_storage/visibility_control.hpp"

namespace rosbag2_storage
{

ROSBAG2_STORAGE_PUBLIC std::string get_default_storage_id();

}  // namespace rosbag2_storage

#endif  // ROSBAG2_STORAGE__DEFAULT_STORAGE_ID_HPP_
