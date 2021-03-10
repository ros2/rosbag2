// Copyright 2021 Open Source Robotics Foundation, Inc.
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

#include <memory>
#include <string>

#include "rosbag2_cpp/info.hpp"
#include "rosbag2_storage/bag_metadata.hpp"

#include "./pybind11.hpp"

namespace rosbag2_py
{

class Info
{
public:
  Info()
  : info_(std::make_unique<rosbag2_cpp::Info>())
  {}

  ~Info() = default;

  rosbag2_storage::BagMetadata read_metadata(
    const std::string & uri, const std::string & storage_id)
  {
    return info_->read_metadata(uri, storage_id);
  }

protected:
  std::unique_ptr<rosbag2_cpp::Info> info_;
};

}  // namespace rosbag2_py

PYBIND11_MODULE(_info, m) {
  m.doc() = "Python wrapper of the rosbag2_cpp info API";

  pybind11::class_<rosbag2_py::Info>(m, "Info")
  .def(pybind11::init())
  .def("read_metadata", &rosbag2_py::Info::read_metadata);
}
