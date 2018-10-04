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

#include "rosbag2/rosbag2_factory.hpp"

#include <memory>

#include "sequential_reader_impl.hpp"
#include "writer_impl.hpp"

namespace rosbag2
{

std::shared_ptr<Writer> Rosbag2Factory::create_writer(const StorageOptions & options)
{
  return std::make_shared<WriterImpl>(options);
}

std::shared_ptr<SequentialReader> Rosbag2Factory::create_sequential_reader(
  const StorageOptions & options)
{
  return std::make_shared<SequentialReaderImpl>(options);
}

std::shared_ptr<Info> Rosbag2Factory::create_info()
{
  return std::make_shared<Info>();
}

}  // namespace rosbag2
