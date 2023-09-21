// Copyright 2023 Open Source Robotics Foundation, Inc.
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

#ifndef ROSBAG2_COMPRESSION__FAKE_COMPRESSION_FACTORY_HPP_
#define ROSBAG2_COMPRESSION__FAKE_COMPRESSION_FACTORY_HPP_

#include <memory>
#include <string>

#include "rosbag2_compression/compression_factory.hpp"
#include "fake_compressor.hpp"

class FakeCompressionFactory
  : public rosbag2_compression::CompressionFactory
{
public:
  FakeCompressionFactory() = delete;

  ~FakeCompressionFactory() override = default;

  explicit FakeCompressionFactory(int & detected_thread_priority)
  : detected_thread_priority_(detected_thread_priority) {}

  std::shared_ptr<rosbag2_compression::BaseCompressorInterface>
  create_compressor(const std::string & /*compression_format*/) override
  {
    return std::make_shared<FakeCompressor>(detected_thread_priority_);
  }

private:
  int & detected_thread_priority_;
};

#endif  // ROSBAG2_COMPRESSION__FAKE_COMPRESSION_FACTORY_HPP_
