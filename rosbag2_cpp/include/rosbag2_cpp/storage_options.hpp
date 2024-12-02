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

#ifndef ROSBAG2_CPP__STORAGE_OPTIONS_HPP_
#define ROSBAG2_CPP__STORAGE_OPTIONS_HPP_

<<<<<<< HEAD:rosbag2_cpp/include/rosbag2_cpp/storage_options.hpp
#include "rosbag2_storage/storage_options.hpp"
=======
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
>>>>>>> 9ec61ea ([jazzy] Add computation of size contribution to info verb (backport #1726) (#1872)):rosbag2_py/src/rosbag2_py/format_service_info.hpp

namespace rosbag2_cpp
{

<<<<<<< HEAD:rosbag2_cpp/include/rosbag2_cpp/storage_options.hpp
using StorageOptions [[deprecated("use rosbag2_storage::StorageOptions instead")]] =
  rosbag2_storage::StorageOptions;
=======
std::string format_service_info(
  std::vector<std::shared_ptr<rosbag2_cpp::rosbag2_service_info_t>> & service_info,
  const std::unordered_map<std::string, uint64_t> & messages_size = {},
  bool verbose = false);
>>>>>>> 9ec61ea ([jazzy] Add computation of size contribution to info verb (backport #1726) (#1872)):rosbag2_py/src/rosbag2_py/format_service_info.hpp

}  // namespace rosbag2_cpp
#endif  // ROSBAG2_CPP__STORAGE_OPTIONS_HPP_
