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

#ifndef ROSBAG2__ROSBAG2_FACTORY_HPP_
#define ROSBAG2__ROSBAG2_FACTORY_HPP_

#include <memory>

#include "rosbag2/info.hpp"
#include "rosbag2/storage_options.hpp"
#include "rosbag2/sequential_reader.hpp"
#include "rosbag2/writer.hpp"
#include "rosbag2/visibility_control.hpp"

namespace rosbag2
{

class ROSBAG2_PUBLIC Rosbag2Factory
{
public:
  /**
   * Open a new writer with the given storage options. The bagfile folder will be created and
   * must not already exist and the storage will be opened for writing. On destruction, the
   * writer will write a metadata yaml file.
   *
   * @param options Storage options including uri and storage identifier
   */
  virtual std::shared_ptr<Writer> create_writer(const StorageOptions & options);

  /**
   * Open a bagfile to read all messages sequentially according to their timestamp.
   *
   * @param options Storage options including uri and storage identifier
   * @return
   */
  virtual std::shared_ptr<SequentialReader> create_sequential_reader(
    const StorageOptions & options);

  virtual std::shared_ptr<Info> create_info();
};

}  // namespace rosbag2

#endif  // ROSBAG2__ROSBAG2_FACTORY_HPP_
