// Copyright 2021 DCS Corporation, All Rights Reserved.
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

#include <memory>
#include <string>
#include <vector>

# include "rosbag2_cpp/reindexer.hpp"
#include "rosbag2_storage/storage_options.hpp"

#include "./pybind11.hpp"

namespace rosbag2_py
{

class Reindexer
{
public:
  Reindexer()
  : reindexer_(std::make_unique<rosbag2_cpp::Reindexer>())
  {
  }

  void reindex(const rosbag2_storage::StorageOptions & storage_options)
  {
    reindexer_->reindex(storage_options);
  }

protected:
  std::unique_ptr<rosbag2_cpp::Reindexer> reindexer_;
};
}  // namespace rosbag2_py

PYBIND11_MODULE(_reindexer, m) {
  m.doc() = "Python wrapper of the rosbag2_cpp reindexer API";

  pybind11::class_<rosbag2_py::Reindexer>(
    m, "Reindexer")
  .def(pybind11::init())
  .def("reindex", &rosbag2_py::Reindexer::reindex);
}
