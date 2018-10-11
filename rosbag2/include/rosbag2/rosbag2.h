// Copyright 2018, Bosch Software Innovations GmbH.
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

#ifndef ROSBAG2__ROSBAG2_H_
#define ROSBAG2__ROSBAG2_H_

#include "rcutils/types.h"
#include "rmw/names_and_types.h"
#include "rosbag2/types/bag_handle.h"
#include "rosbag2/types/bag_metadata.h"
#include "rosbag2/types/serialized_bag_message.h"
#include "rosbag2/types/sequential_reader.h"
#include "rosbag2/types/writer.h"

extern "C"
{

int rosbag2_info(rcutils_char_array_t * uri, rosbag2_bag_metadata_t * bag_metadata);

};

#endif  // ROSBAG2__ROSBAG2_H_
